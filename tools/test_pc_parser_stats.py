#!/usr/bin/env python3
"""Lightweight checks for PC CDC parser diagnostic counters."""

from __future__ import annotations

from plot_everything import AcquisitionModel, missing_pc_frames


def main() -> int:
    assert missing_pc_frames(None, 10) == 0
    assert missing_pc_frames(10, 11) == 0
    assert missing_pc_frames(10, 14) == 3
    assert missing_pc_frames(254, 255) == 0
    assert missing_pc_frames(255, 0) == 0
    assert missing_pc_frames(250, 3) == 8
    assert missing_pc_frames(7, 7) == 0

    model = AcquisitionModel(max_points=32, emg_display_rate=2000.0)
    model.update_diag(1000, 2, 2, link_events_per_sec=3, queue_drops_per_sec=4, ack_fails_per_sec=1)
    model.update_diag(1001, 0, 2, link_events_per_sec=5, queue_drops_per_sec=0, ack_fails_per_sec=2)
    model.note_bad_count(99)
    model.note_pc_sequence_gap(4)
    model.note_pc_resend_request(2)
    model.note_pc_resend_recovered()
    model.note_pc_duplicate_frame()
    model.note_crc_fail()
    model.note_resync_bytes(123)
    model.note_host_queue_drop(8192)

    *_, parser_stats = model.diag_snapshot()
    snapshot = model.diag_snapshot()
    assert snapshot[6:10] == (2, 8, 4, 3)
    assert parser_stats.bad_counts == 1
    assert parser_stats.last_bad_count == 99
    assert parser_stats.pc_sequence_gaps == 4
    assert parser_stats.pc_resend_requests == 2
    assert parser_stats.pc_resend_recovered == 1
    assert parser_stats.pc_duplicate_frames == 1
    assert parser_stats.crc_fails == 1
    assert parser_stats.resync_bytes == 123
    assert parser_stats.host_queue_drops == 1
    assert parser_stats.host_queue_drop_bytes == 8192
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
