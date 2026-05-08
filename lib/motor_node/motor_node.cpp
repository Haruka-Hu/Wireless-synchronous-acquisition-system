#include "motor_node.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

namespace {

MotorApp *g_activeMotorApp = nullptr;
BLECharacteristic *g_txCharacteristic = nullptr;
BLEAdvertising *g_advertising = nullptr;

// BLE 库要求回调类；这里仅转发到 MotorApp，不在回调中直接控制电机。
class ServerCallbacks : public BLEServerCallbacks {
  // BLE 中心设备连接时通知应用层更新连接状态。
  void onConnect(BLEServer *) override {
    if (g_activeMotorApp != nullptr) {
      g_activeMotorApp->onBleConnect();
    }
  }

  // BLE 中心设备断开时通知应用层停机并恢复广播。
  void onDisconnect(BLEServer *) override {
    if (g_activeMotorApp != nullptr) {
      g_activeMotorApp->onBleDisconnect();
    }
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  // 收到 RX characteristic 写入时，把原始文本交给应用层按行解析。
  void onWrite(BLECharacteristic *characteristic) override {
    if (g_activeMotorApp != nullptr) {
      g_activeMotorApp->onBleWrite(characteristic->getValue().c_str());
    }
  }
};

}  // namespace

// 保存 BLE、电机和档位参数，并构造底层 BTS7960 驱动。
MotorApp::MotorApp(const Config &config) : config_(config), motor_(config.motor) {}

// 初始化串口、命令队列、电机驱动和 BLE GATT 服务。
void MotorApp::begin() {
  g_activeMotorApp = this;
  Serial.begin(config_.serialBaud);
  delay(300);
  Serial.println();
  Serial.println("Neuro Hammer motor node starting...");

  // BLE 写入先进入命令队列，loopOnce 再处理，保证 UI 连续写命令时不会卡住 BLE 回调。
  commandQueue_ = xQueueCreate(12, sizeof(MotorCommand));
  motor_.begin();
  stopMotor();
  Serial.println("Motor driver ready.");
  initBle();
}

// 处理队列中的所有待执行命令，并推进电机非阻塞状态机。
void MotorApp::loopOnce() {
  // 一次循环先清空命令队列，再推进当前非阻塞叩击状态机。
  MotorCommand command{};
  while (commandQueue_ != nullptr && xQueueReceive(commandQueue_, &command, 0) == pdTRUE) {
    handleCommand(command);
  }

  updateMotorState();
  delay(2);
}

// 静态命令入口，供 BLE 回调在没有 this 指针的情况下投递命令。
void MotorApp::enqueueCommandStatic(const MotorCommand &command) {
  if (g_activeMotorApp != nullptr) {
    g_activeMotorApp->enqueueCommand(command);
  }
}

// 把上位机文本命令解析成内部命令枚举和档位号。
MotorApp::MotorCommand MotorApp::parseCommand(String raw) {
  // 上位机命令按行传输，固件端统一忽略大小写和首尾空白。
  raw.trim();
  raw.toUpperCase();

  if (raw == "PING") {
    return {CommandType::Ping, 0};
  }
  if (raw == "STOP") {
    return {CommandType::Stop, 0};
  }
  if (raw == "MOTOR_FWD") {
    return {CommandType::ManualForward, 0};
  }
  if (raw == "MOTOR_REV") {
    return {CommandType::ManualReverse, 0};
  }
  if (raw.startsWith("STRIKE_") && raw.length() == 8) {
    const int gear = raw.substring(7).toInt();
    if (gear >= 1 && gear <= 5) {
      return {CommandType::Strike, static_cast<uint8_t>(gear)};
    }
  }

  return {CommandType::Unknown, 0};
}

// 创建 Nordic UART 风格 BLE 服务，并开始广播设备名。
void MotorApp::initBle() {
  // 使用 Nordic UART 风格的 RX/TX characteristic：RX 接收命令，TX notify 返回 ACK/ERR。
  BLEDevice::init(config_.ble.deviceName);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(config_.ble.serviceUuid);

  g_txCharacteristic = service->createCharacteristic(
      config_.ble.txUuid,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  g_txCharacteristic->addDescriptor(new BLE2902());
  g_txCharacteristic->setValue("READY\n");

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      config_.ble.rxUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  g_advertising = BLEDevice::getAdvertising();
  g_advertising->addServiceUUID(config_.ble.serviceUuid);
  g_advertising->setScanResponse(true);
  g_advertising->setMinPreferred(0x06);
  g_advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.print("BLE advertising as ");
  Serial.print(config_.ble.deviceName);
  Serial.println(".");
}

// 将 BLE 或内部产生的命令放入 FreeRTOS 队列。
void MotorApp::enqueueCommand(const MotorCommand &command) {
  if (commandQueue_ == nullptr) {
    return;
  }
  xQueueSend(commandQueue_, &command, 0);
}

// 通过串口和 BLE TX notify 向上位机返回 ACK、ERR 或状态消息。
void MotorApp::notifyHost(const String &message) {
  // 串口和 BLE 同时输出，便于无上位机时也能用串口观察控制状态。
  Serial.println(message);
  if (!bleConnected_ || g_txCharacteristic == nullptr) {
    return;
  }
  const String line = message + "\n";
  g_txCharacteristic->setValue(reinterpret_cast<uint8_t *>(const_cast<char *>(line.c_str())), line.length());
  g_txCharacteristic->notify();
}

// 立即停止电机，并清空当前叩击或手动转动状态。
void MotorApp::stopMotor() {
  // STOP 是最高优先级动作：立即停 PWM，并清掉当前叩击阶段。
  motor_.stop();
  mode_ = MotorMode::Idle;
  activeGear_ = 0;
  phaseDeadlineMs_ = 0;
}

// 启动指定档位的一次完整叩击，先进入回撤阶段。
void MotorApp::beginStrike(uint8_t gear) {
  // 五档叩击共享同一套两阶段时序，只从查表参数改变 PWM 和持续时间。
  const StrikeProfile &profile = config_.strikeProfiles[gear - 1];
  mode_ = MotorMode::StrikePull;
  activeGear_ = gear;
  phaseDeadlineMs_ = millis() + profile.pullMs;
  motor_.driveReverse(profile.pullPwm);
  notifyHost("ACK: STRIKE_" + String(gear));
}

// 进入固定 PWM 正转模式，直到收到 STOP 或 BLE 断开。
void MotorApp::beginManualForward() {
  mode_ = MotorMode::ManualForward;
  activeGear_ = 0;
  phaseDeadlineMs_ = 0;
  motor_.driveForward(config_.manualPwm);
  notifyHost("ACK: MOTOR_FWD");
}

// 进入固定 PWM 反转模式，直到收到 STOP 或 BLE 断开。
void MotorApp::beginManualReverse() {
  mode_ = MotorMode::ManualReverse;
  activeGear_ = 0;
  phaseDeadlineMs_ = 0;
  motor_.driveReverse(config_.manualPwm);
  notifyHost("ACK: MOTOR_REV");
}

// 执行解析后的命令，并向上位机返回对应 ACK/ERR。
void MotorApp::handleCommand(const MotorCommand &command) {
  switch (command.type) {
    case CommandType::Ping:
      notifyHost("ACK: PONG");
      break;
    case CommandType::Stop:
      stopMotor();
      notifyHost("ACK: STOP");
      break;
    case CommandType::Strike:
      if (command.gear < 1 || command.gear > config_.strikeProfileCount) {
        notifyHost("ERR: INVALID_GEAR");
      } else {
        beginStrike(command.gear);
      }
      break;
    case CommandType::ManualForward:
      beginManualForward();
      break;
    case CommandType::ManualReverse:
      beginManualReverse();
      break;
    case CommandType::Unknown:
    default:
      notifyHost("ERR: UNKNOWN_COMMAND");
      break;
  }
}

// 根据 millis() 推进叩击状态机；手动模式不在这里自动退出。
void MotorApp::updateMotorState() {
  // 手动正反转不靠定时退出，只能由 STOP 或断开连接停止。
  if (mode_ != MotorMode::StrikePull && mode_ != MotorMode::StrikeFire) {
    return;
  }

  if (static_cast<int32_t>(millis() - phaseDeadlineMs_) < 0) {
    return;
  }

  const StrikeProfile &profile = config_.strikeProfiles[activeGear_ - 1];
  if (mode_ == MotorMode::StrikePull) {
    // 回撤时间到后切到正向敲击。
    mode_ = MotorMode::StrikeFire;
    phaseDeadlineMs_ = millis() + profile.strikeMs;
    motor_.driveForward(profile.strikePwm);
    return;
  }

  const uint8_t finishedGear = activeGear_;
  stopMotor();
  notifyHost("ACK: STRIKE_DONE_" + String(finishedGear));
}

// BLE 连接建立后标记 TX notify 可用。
void MotorApp::onBleConnect() {
  bleConnected_ = true;
  Serial.println("BLE client connected.");
}

// BLE 断开后立即停机并重新开始广播。
void MotorApp::onBleDisconnect() {
  // 蓝牙断开时立即停机，避免按住按钮期间链路断开导致电机保持转动。
  bleConnected_ = false;
  stopMotor();
  Serial.println("BLE client disconnected.");
  if (g_advertising != nullptr) {
    g_advertising->start();
  }
}

// 接收 BLE RX 文本，将其中的一行或多行命令拆分后入队。
void MotorApp::onBleWrite(const String &rawValue) {
  // 同一次 BLE 写入可能包含多行命令，逐行拆分后分别入队。
  int start = 0;
  for (int i = 0; i <= rawValue.length(); ++i) {
    if (i != rawValue.length() && rawValue[i] != '\n' && rawValue[i] != '\r') {
      continue;
    }
    if (i > start) {
      enqueueCommand(parseCommand(rawValue.substring(start, i)));
    }
    start = i + 1;
  }
}
