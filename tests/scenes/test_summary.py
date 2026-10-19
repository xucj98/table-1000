"""The scene summary: each step rewrites only its section; batch runs split seeds evenly."""

import json

from scripts.scenes.generate import split
from table_1000.scenes import summary
from table_1000.scenes.recordings import demo_dir, video_path, write_render


def test_seeds_split_into_contiguous_parts():
    assert split(list(range(10)), 3) == [[0, 1, 2, 3], [4, 5, 6], [7, 8, 9]]
    assert split([5, 6], 4) == [[5], [6]]


def test_sections_are_updated_independently(tmp_path):
    summary.update(tmp_path, '000000', 'initial', {'valid': 25})
    summary.update(tmp_path, '000000', 'generation', {'succeeded': 98}, key='isaac-cpu')
    file = summary.update(tmp_path, '000000', 'initial', {'valid': 24})
    data = json.loads(file.read_text())
    assert data == {'scene': '000000', 'initial': {'valid': 24}, 'generation': {'isaac-cpu': {'succeeded': 98}}}


def test_initial_section_reports_the_largest_settling_motion():
    attempt = {'settle': {'objects': {'pen': {'move_m': .0004, 'turn_rad': .003}}, 'robot_contacts': []}}
    report = {'view': 'top', 'valid': 1, 'invalid': 1, 'mean_attempts': 2.,
              'seeds': {0: {'attempt': 1, 'attempts': [{'settle': None}, attempt]},
                        1: {'attempt': None, 'attempts': [{'settle': None}]}}}
    section = summary.initial_section(report)
    assert section['without_state'] == [1]
    assert round(section['settle']['pen']['max_move_mm'], 3) == .4


def test_preview_section_lists_rendered_episodes(tmp_path):
    for name in ['demo_10', 'demo_2']:
        demo_dir(tmp_path, name).mkdir(parents=True)
        (demo_dir(tmp_path, name) / 'sheet.jpg').touch()
    video_path(tmp_path, 'demo_10').touch()
    (demo_dir(tmp_path, 'demo_10') / 'frames' / 'face').mkdir(parents=True)
    write_render(demo_dir(tmp_path, 'demo_10'), {'sheet': True, 'views': ['wide'], 'cameras': ['face']})
    write_render(demo_dir(tmp_path, 'demo_2'), {'sheet': True})
    section = summary.preview_section(tmp_path, {'demo_10', 'demo_2'})
    assert section['sheets'] == 2 and section['videos'] == ['demo_10'] and section['camera_frames'] == ['demo_10']
    assert section['output'] == str(tmp_path)
