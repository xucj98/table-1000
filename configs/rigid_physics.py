"""Physics of the assets simulated as one rigid body that have no ``physics.py`` of their own.

``build_physics.py`` authors them with :func:`table_1000.physics.rigid.author`: ``mass_kg`` in total,
``friction`` ``(static, dynamic)``, ``angular_damping`` (1/s) for round things that would roll on, and
``center_of_mass`` (m, in the body frame) where the colliders' uniform density puts it wrong (a heavy
base under light reeds, arms or flowers). Values are design estimates, not calibration. An asset with
its own ``physics.py`` (drawers, snap fits) is a detailed physics asset and is not listed here.
"""

ASSETS = {
    'battery/000000': {'mass_kg': 0.023, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # metal
    'bench_backboard/000000': {'mass_kg': 12.0, 'friction': (0.6, 0.5), 'center_of_mass': (0.0, -0.015, 0.06)},  # heavy base
    'bench_power_supply/000000': {'mass_kg': 4.0, 'friction': (0.6, 0.5)},
    'binder_clip/000000': {'mass_kg': 0.004, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000001': {'mass_kg': 0.004, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000002': {'mass_kg': 0.012, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000003': {'mass_kg': 0.012, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000004': {'mass_kg': 0.004, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000005': {'mass_kg': 0.004, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000006': {'mass_kg': 0.002, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000007': {'mass_kg': 0.002, 'friction': (0.3, 0.25)},  # metal
    'binder_clip/000008': {'mass_kg': 0.002, 'friction': (0.3, 0.25)},  # metal
    'board_eraser/000000': {'mass_kg': 0.06, 'friction': (0.4, 0.3)},  # plastic
    'cable_coil/000000': {'mass_kg': 0.07, 'friction': (0.5, 0.45)},  # PVC cable
    'cable_coil/000001': {'mass_kg': 0.09, 'friction': (0.5, 0.45)},  # PVC cable
    'calculator/000000': {'mass_kg': 0.12, 'friction': (0.5, 0.4)},  # plastic
    'calculator/000001': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # plastic
    'circuit_board/000000': {'mass_kg': 0.05, 'friction': (0.5, 0.4)},  # FR4 on nylon feet
    'circuit_board/000001': {'mass_kg': 0.01, 'friction': (0.5, 0.4)},  # FR4 on nylon feet
    'clip_lead/000000': {'mass_kg': 0.018, 'friction': (0.6, 0.5)},
    'clip_lead/000001': {'mass_kg': 0.018, 'friction': (0.6, 0.5)},
    'comb/000000': {'mass_kg': 0.02, 'friction': (0.45, 0.35)},  # plastic
    'compact_mirror/000000': {'mass_kg': 0.08, 'friction': (0.4, 0.3), 'angular_damping': 1.0},  # metal
    'computer_mouse/000000': {'mass_kg': 0.09, 'friction': (0.4, 0.35)},  # plastic
    'computer_mouse/000001': {'mass_kg': 0.085, 'friction': (0.4, 0.35)},  # plastic
    'cosmetic_cup/000000': {'mass_kg': 0.15, 'friction': (0.45, 0.35)},  # plastic
    'cotton_pad_jar/000000': {'mass_kg': 0.15, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'cream_jar/000000': {'mass_kg': 0.15, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'cream_jar/000001': {'mass_kg': 0.12, 'friction': (0.5, 0.4), 'angular_damping': 2.0},  # glass/painted wood
    'db_adapter/000000': {'mass_kg': 0.035, 'friction': (0.4, 0.35)},  # plastic and steel
    'deodorant/000000': {'mass_kg': 0.12, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'desk_lamp/000000': {'mass_kg': 2.5, 'friction': (0.6, 0.5), 'center_of_mass': (0.0, -0.03, 0.04)},  # heavy base
    'desolder_wick/000000': {'mass_kg': 0.012, 'friction': (0.5, 0.4)},
    'distractor_food/000000': {'mass_kg': 0.03, 'friction': (0.6, 0.5)},  # food packaging
    'drink_can/000000': {'mass_kg': 0.35, 'friction': (0.35, 0.3)},  # aluminium
    'drink_can/000001': {'mass_kg': 0.015, 'friction': (0.35, 0.3)},  # aluminium, empty
    'electrical_tape/000000': {'mass_kg': 0.04, 'friction': (0.6, 0.5)},  # PVC
    'eraser/000000': {'mass_kg': 0.015, 'friction': (0.8, 0.7)},  # rubber
    'file_basket/000000': {'mass_kg': 0.25, 'friction': (0.4, 0.3)},  # plastic
    'flower_vase/000000': {'mass_kg': 0.45, 'friction': (0.5, 0.4), 'center_of_mass': (0.0, 0.0, 0.042)},  # heavy base
    'flux_tin/000000': {'mass_kg': 0.035, 'friction': (0.4, 0.3)},
    'glue/000000': {'mass_kg': 0.03, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'glue/000001': {'mass_kg': 0.04, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'hairbrush/000000': {'mass_kg': 0.1, 'friction': (0.45, 0.35)},  # plastic
    'heat_sink/000000': {'mass_kg': 0.12, 'friction': (0.45, 0.35)},
    'helping_hands/000000': {'mass_kg': 0.45, 'friction': (0.6, 0.5), 'center_of_mass': (0.0, 0.004, 0.01)},  # heavy base
    'instrument_box/000000': {'mass_kg': 1.2, 'friction': (0.6, 0.5)},
    'iron_holder/000000': {'mass_kg': 0.45, 'friction': (0.6, 0.5), 'center_of_mass': (-0.002, 0.004, 0.009)},  # heavy base
    'keyboard/000000': {'mass_kg': 0.9, 'friction': (0.5, 0.4)},  # plastic
    'lab_notebook/000000': {'mass_kg': 0.25, 'friction': (0.5, 0.4)},  # paper
    'laptop/000000': {'mass_kg': 1.37, 'friction': (0.7, 0.6)},  # rubber feet
    'lcd_module/000000': {'mass_kg': 0.012, 'friction': (0.4, 0.35)},  # FR4 and glass
    'lipstick/000000': {'mass_kg': 0.03, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'lipstick/000001': {'mass_kg': 0.03, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'lipstick/000002': {'mass_kg': 0.03, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'lotion_bottle/000000': {'mass_kg': 0.3, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'magnet/000000': {'mass_kg': 0.006, 'friction': (0.4, 0.3)},  # plastic
    'magnet/000001': {'mass_kg': 0.006, 'friction': (0.4, 0.3)},  # plastic
    'magnet/000002': {'mass_kg': 0.006, 'friction': (0.4, 0.3)},  # plastic
    'makeup_brush/000000': {'mass_kg': 0.025, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'makeup_brush/000001': {'mass_kg': 0.012, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'monitor/000000': {'mass_kg': 3.5, 'friction': (0.6, 0.5), 'center_of_mass': (0.0, 0.01, 0.09)},  # heavy base
    'multimeter/000000': {'mass_kg': 0.4, 'friction': (0.8, 0.7)},  # rubber
    'multimeter/000001': {'mass_kg': 0.18, 'friction': (0.8, 0.7)},  # rubber
    'nail_polish/000000': {'mass_kg': 0.05, 'friction': (0.5, 0.4)},  # glass/painted wood
    'nail_polish/000001': {'mass_kg': 0.05, 'friction': (0.5, 0.4)},  # glass/painted wood
    'nail_polish/000002': {'mass_kg': 0.05, 'friction': (0.5, 0.4)},  # glass/painted wood
    'notebook/000000': {'mass_kg': 0.15, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000001': {'mass_kg': 0.15, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000002': {'mass_kg': 0.35, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000003': {'mass_kg': 0.35, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000004': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000005': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # paper cover
    'notebook/000006': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # paper cover
    'oil_bottle/000000': {'mass_kg': 0.18, 'friction': (0.5, 0.4)},  # glass/painted wood
    'oscilloscope/000000': {'mass_kg': 3.0, 'friction': (0.6, 0.5)},
    'paper_label/000000': {'mass_kg': 0.0005, 'friction': (0.5, 0.45)},  # paper
    'paper_label/000001': {'mass_kg': 0.0005, 'friction': (0.5, 0.45)},  # paper
    'paper_label/000002': {'mass_kg': 0.0005, 'friction': (0.5, 0.45)},  # paper
    'paper_label/000003': {'mass_kg': 0.0005, 'friction': (0.5, 0.45)},  # paper
    'parts_box/000000': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # plastic
    'pen/000000': {'mass_kg': 0.012, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000001': {'mass_kg': 0.01, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000003': {'mass_kg': 0.02, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000004': {'mass_kg': 0.01, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000006': {'mass_kg': 0.02, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000007': {'mass_kg': 0.015, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000008': {'mass_kg': 0.015, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000009': {'mass_kg': 0.01, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000010': {'mass_kg': 0.008, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000011': {'mass_kg': 0.012, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'pen/000012': {'mass_kg': 0.012, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # metal and plastic
    'pen_holder/000000': {'mass_kg': 0.15, 'friction': (0.4, 0.3)},  # plastic
    'pen_holder/000001': {'mass_kg': 0.12, 'friction': (0.3, 0.25)},  # metal
    'pencil_case/000000': {'mass_kg': 0.08, 'friction': (0.5, 0.4)},  # paper cover
    'perfume_bottle/000000': {'mass_kg': 0.3, 'friction': (0.5, 0.4)},  # glass/painted wood
    'perfume_bottle/000001': {'mass_kg': 0.32, 'friction': (0.5, 0.4), 'angular_damping': 2.0},  # glass/painted wood
    'perfume_bottle/000002': {'mass_kg': 0.18, 'friction': (0.5, 0.4)},  # glass/painted wood
    'perfume_bottle/000003': {'mass_kg': 0.15, 'friction': (0.5, 0.4), 'angular_damping': 2.0},  # glass/painted wood
    'plastic_scrap/000000': {'mass_kg': 0.0002, 'friction': (0.4, 0.35)},  # plastic
    'pliers/000000': {'mass_kg': 0.12, 'friction': (0.4, 0.35)},  # steel and plastic
    'pliers/000001': {'mass_kg': 0.1, 'friction': (0.4, 0.35)},  # steel and plastic
    'power_adapter/000000': {'mass_kg': 0.25, 'friction': (0.5, 0.4)},  # plastic
    'power_plug/000000': {'mass_kg': 0.05, 'friction': (0.4, 0.3)},  # plastic
    'precision_screwdriver/000000': {'mass_kg': 0.03, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'precision_screwdriver/000001': {'mass_kg': 0.03, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'receipt/000000': {'mass_kg': 0.001, 'friction': (0.5, 0.45)},  # paper
    'reed_diffuser/000000': {'mass_kg': 0.35, 'friction': (0.5, 0.4), 'center_of_mass': (0.0, 0.0, 0.032)},  # heavy base
    'remote_control/000000': {'mass_kg': 0.07, 'friction': (0.4, 0.3)},  # plastic
    'scope_probe/000000': {'mass_kg': 0.04, 'friction': (0.6, 0.5), 'angular_damping': 1.0},
    'screwdriver/000000': {'mass_kg': 0.05, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'screwdriver_set/000000': {'mass_kg': 0.16, 'friction': (0.4, 0.3)},
    'serum_bottle/000000': {'mass_kg': 0.1, 'friction': (0.45, 0.35), 'angular_damping': 2.0},  # plastic
    'solder_spool/000000': {'mass_kg': 0.1, 'friction': (0.4, 0.35)},  # plastic
    'soldering_iron/000000': {'mass_kg': 0.06, 'friction': (0.5, 0.4), 'angular_damping': 2.0},  # plastic
    'soldering_station/000000': {'mass_kg': 1.4, 'friction': (0.6, 0.5)},
    'staple/000000': {'mass_kg': 0.01, 'friction': (0.3, 0.25)},  # metal
    'stapler/000000': {'mass_kg': 0.12, 'friction': (0.4, 0.3)},  # plastic
    'storage_box/000000': {'mass_kg': 0.3, 'friction': (0.35, 0.3)},  # glass
    'storage_box/000001': {'mass_kg': 0.12, 'friction': (0.35, 0.3)},  # glass
    'storage_box/000002': {'mass_kg': 0.05, 'friction': (0.35, 0.3)},  # glass
    'tape/000000': {'mass_kg': 0.04, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'test_probe/000000': {'mass_kg': 0.012, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'test_probe/000001': {'mass_kg': 0.012, 'friction': (0.4, 0.3), 'angular_damping': 2.0},  # plastic
    'tip_cleaner/000000': {'mass_kg': 0.12, 'friction': (0.6, 0.5)},  # plastic
    'tissue_box/000000': {'mass_kg': 0.2, 'friction': (0.5, 0.4)},  # ceramic/card
    'tool_cup/000000': {'mass_kg': 0.15, 'friction': (0.4, 0.35)},  # metal
    'tool_tray/000000': {'mass_kg': 0.45, 'friction': (0.5, 0.4)},  # plastic
    'trash_bin/000000': {'mass_kg': 0.25, 'friction': (0.5, 0.4)},  # plastic
    'trinket_dish/000000': {'mass_kg': 0.1, 'friction': (0.5, 0.4)},  # ceramic/card
    'tweezers/000000': {'mass_kg': 0.015, 'friction': (0.3, 0.25)},  # stainless steel
    'usb_drive/000000': {'mass_kg': 0.01, 'friction': (0.4, 0.3)},  # plastic
    'utility_knife/000000': {'mass_kg': 0.03, 'friction': (0.4, 0.3)},  # plastic
    'vanity_tray/000000': {'mass_kg': 0.6, 'friction': (0.5, 0.4)},  # glass/painted wood
    'water_bottle/000000': {'mass_kg': 0.52, 'friction': (0.4, 0.35)},  # PET
    'water_bottle/000001': {'mass_kg': 0.025, 'friction': (0.4, 0.35)},  # PET, empty
    'wire_scrap/000000': {'mass_kg': 0.002, 'friction': (0.5, 0.4)},  # PVC-insulated copper
    'wire_scrap/000001': {'mass_kg': 0.001, 'friction': (0.5, 0.4)},  # PVC-insulated copper
    'wire_stripper/000000': {'mass_kg': 0.15, 'friction': (0.4, 0.35)},  # steel and plastic
    'zip_bag/000000': {'mass_kg': 0.0015, 'friction': (0.4, 0.35)},  # polyethylene film
}
