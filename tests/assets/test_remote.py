from table_1000.assets import remote


def test_lock_names_a_pinned_revision():
    lock = remote.read_lock()
    assert lock['repo_type'] == 'dataset' and len(lock['revision']) == 40
    assert lock['paths'] == sorted(set(lock['paths']))


def test_patterns_select_whole_directories():
    assert remote.patterns(['robots/x1pro/raw/ex001_render/']) == ['robots/x1pro/raw/ex001_render/**']


def test_write_lock_deduplicates(tmp_path):
    path = tmp_path / 'lock.json'
    remote.write_lock({'repo': 'r', 'repo_type': 'dataset', 'revision': 'x', 'paths': ['b', 'a', 'b']}, path)
    assert remote.read_lock(path)['paths'] == ['a', 'b']
