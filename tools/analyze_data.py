from __future__ import annotations

import argparse

import pandas as pd

def analyze_transmission_stability(csv_file_path):
    # 1. 加载数据
    print(f"正在加载数据: {csv_file_path}")
    df = pd.read_csv(csv_file_path)
    
    # 获取所有的信号源 (如 EMG, Accel, Gyro)
    sources = df['source'].unique()
    print(f"数据集中检测到的信号源: {sources}\n")
    
    results = {}
    
    # 2. 分别对每个数据源进行稳定性分析
    for source in sources:
        # 筛选特定传感器数据并按采样序列号排序
        source_df = df[df['source'] == source].sort_values('sample_seq').reset_index(drop=True)
        
        # --- A. 丢包率分析 (Packet Loss) ---
        # 计算相邻 sample_seq 的差值。正常应该为 1，大于 1 说明有丢包
        seq_diff = source_df['sample_seq'].diff()
        lost_packets = seq_diff[seq_diff > 1] - 1
        total_lost = lost_packets.sum()
        total_samples = len(source_df)
        packet_loss_rate = total_lost / (total_samples + total_lost) if (total_samples + total_lost) > 0 else 0
        
        # --- B. 主机接收时间抖动 (Host RX Jitter) ---
        # 转换为毫秒 (ms) 方便阅读
        rx_diff_ms = source_df['host_rx_time'].diff() * 1000 
        rx_jitter_std = rx_diff_ms.std()
        rx_diff_mean = rx_diff_ms.mean()
        
        # --- C. 传感器时间戳抖动 (Sensor Timestamp Jitter) ---
        # 微秒 (us) 转换为毫秒 (ms)
        ts_diff_ms = source_df['timestamp_us'].diff() / 1000 
        ts_jitter_std = ts_diff_ms.std()
        ts_diff_mean = ts_diff_ms.mean()
        
        # 记录结果
        results[source] = {
            'total_samples': total_samples,
            'total_lost_packets': int(total_lost) if pd.notna(total_lost) else 0,
            'packet_loss_rate': packet_loss_rate,
            'mean_rx_interval_ms': rx_diff_mean,
            'std_rx_interval_ms': rx_jitter_std,
            'mean_sensor_interval_ms': ts_diff_mean,
            'std_sensor_interval_ms': ts_jitter_std
        }
        
        # 打印控制台报告
        print(f"=== {source} 传输稳定性报告 ===")
        print(f"总接收包数: {total_samples} | 预估丢包数: {results[source]['total_lost_packets']}")
        print(f"丢包率 (Loss Rate): {packet_loss_rate:.2%}")
        print(f"主机接收间隔 (Host RX): 平均 {rx_diff_mean:.2f} ms | 抖动(Std) {rx_jitter_std:.2f} ms")
        print(f"传感器采样间隔 (Sensor TS): 平均 {ts_diff_mean:.2f} ms | 抖动(Std) {ts_jitter_std:.2f} ms\n")
        
    print("分析完成。")
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="分析采集 CSV 的 sample_seq 丢包和时间抖动。")
    parser.add_argument("csv_file", help="要分析的 imu_capture_*.csv 文件。")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    analyze_transmission_stability(args.csv_file)
