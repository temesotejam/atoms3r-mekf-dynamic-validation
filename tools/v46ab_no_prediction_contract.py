"""Reverse V46ab autonomous no-prediction changes for retained legacy hashes."""
import re

OLD_CONTROL_ASSIGN = """    status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_;"""
OLD_DETECTOR_UPDATE = """  // V46aa control-zero update begin
  status_.pitch_mekf_detector_relative_deg =
      isfinite(status_.mekf_detector_zero_predicted_abs_deg)
          ? raw_mekf_predicted_abs_deg_ - status_.mekf_detector_zero_predicted_abs_deg
          : NAN;
  // V46aa control-zero update end"""

def normalize_runner(text: str) -> str:
    text = re.sub(
        r"\n\s*// V46ab control-angle begin\n.*?\n\s*// V46ab control-angle end",
        "\n" + OLD_CONTROL_ASSIGN,
        text,
        flags=re.S,
    )
    text = re.sub(
        r"\n\s*// V46ab detector-angle begin\n.*?\n\s*// V46ab detector-angle end",
        "\n" + OLD_DETECTOR_UPDATE,
        text,
        flags=re.S,
    )
    return text

def normalize_config(text: str) -> str:
    return text.replace(
        "v46ab_no_control_prediction_20260918",
        "v46aa_control_upright_zero_20260918",
    )
