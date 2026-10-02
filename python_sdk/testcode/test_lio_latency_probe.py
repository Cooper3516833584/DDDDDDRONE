from lio_latency_probe import ExactStampPairs, LatencyStats


def test_latency_thresholds_and_timestamp_failures():
    stats = LatencyStats()
    for stamp, delay in ((200, 20), (201, 50), (202, 51), (199, 101), (199, -1)):
        stats.add(stamp, stamp + delay * 1000000)
    result = stats.report()
    assert result["count"] == 5
    assert result["min_ms"] == -1
    assert result["median_ms"] == 50
    assert result["max_ms"] == 101
    assert result["late_50ms_count"] == 2
    assert result["late_100ms_count"] == 1
    assert result["timestamp_rewind_count"] == 1
    assert result["timestamp_duplicate_count"] == 1
    assert result["negative_latency_count"] == 1


def test_pairs_require_exact_stamps_and_either_arrival_order():
    pairs = ExactStampPairs(capacity=2)
    pairs.add(0, 100)
    pairs.add(1, 101)
    assert pairs.report()["paired_count"] == 0
    pairs.add(1, 100)
    pairs.add(0, 101)
    assert pairs.report()["paired_count"] == 2
    assert pairs.report()["unpaired_count"] == 0
    for stamp in (102, 103, 104):
        pairs.add(0, stamp)
    assert pairs.report()["evicted_unpaired_count"] == 1
    assert pairs.report()["unpaired_count"] == 3


def test_quantile_buffer_is_bounded_without_losing_total_counters():
    stats = LatencyStats(max_samples=2)
    for stamp in range(3):
        stats.add(stamp, stamp + 100000000)
    assert stats.report()["count"] == 3
    assert stats.report()["quantile_sample_count"] == 2
    assert stats.report()["late_50ms_count"] == 3
