"""Localization rendering overrides never change mapping or estimator inputs."""
from mocap_anchored_mapping.launching import native_output_overrides


def test_mapping_retains_all_original_outputs():
    assert native_output_overrides('mapping')==[]


def test_localization_disables_unused_rendering_outputs_only():
    assert native_output_overrides('localization')==[
        {'pcd_save.pcd_save_en':False,'publish.map_en':False,
         'publish.scan_publish_en':False,'publish.scan_bodyframe_pub_en':False,
         'publish.path_en':False}]
