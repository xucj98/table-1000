"""Acceptance bookkeeping of :class:`~table_1000.robots.checks.Checker` (no simulator)."""

from types import SimpleNamespace

from table_1000.robots.checks import Checker


class Contacts:
    def __init__(self):
        self.entries = []

    def intervals(self):
        return list(self.entries)


def checker():
    """A checker whose environment reset clears the backend's contacts, as the simulator does."""
    contacts = Contacts()
    check = Checker.__new__(Checker)
    check.backend = SimpleNamespace(contacts=contacts)
    check.env = SimpleNamespace(initial_robot_q=dict, reset=lambda layout, q: contacts.entries.clear())
    check.scene = SimpleNamespace(placement=lambda **poses: poses)
    return check, contacts


def test_common_checks_cover_every_sub_case():
    """A deep contact and fast motion in the first of two sub-cases still fail the case."""
    check, contacts = checker()

    def two_objects():
        check.reset()
        contacts.entries.append({'pair': ('robot:finger', 'box:body'), 'max_penetration_m': .02})
        check.trace.update(joint_speed=3., joint='joint6', object_speed=4.)
        first = check.penetration(sub_case=True)
        check.reset()
        contacts.entries.append({'pair': ('robot:finger', 'stick:body'), 'max_penetration_m': .0001})
        return check.common({'objects': {'box': first, 'stick': check.penetration(sub_case=True)}})

    check.two_objects = two_objects
    result = check.run('two_objects')
    assert result['objects'] == {'box': .02, 'stick': .0001}
    assert result['max_penetration_m'] == .02
    assert not any(result['checks'].values()) and result['status'] == 'FAIL'
    assert 'robot:finger / box:body' in result['contacts']
