"""Reverse V46ac autonomous light delay compensation for retained hashes."""
import re

def normalize_runner(text: str) -> str:
    text = re.sub(
        r"\n\s*// V46ac autonomous diagnostic prediction begin\n.*?\n\s*// V46ac autonomous diagnostic prediction end",
        "",
        text,
        flags=re.S,
    )
    return re.sub(
        r"\n\s*// V46ac delay compensation begin\n.*?\n\s*// V46ac delay compensation end",
        """
  // V46ab no-control-prediction begin
  // Autonomous control and video comparison use the exact same posterior,
  // measurement-start-relative coordinate. Predicted MEKF remains diagnostic only.
  if (energy_control_autonomous_mode_) {
    status_.pitch_mekf_deg = status_.pitch_mekf_measurement_relative_deg;
  }
  status_.pitch_mekf_detector_relative_deg =
      status_.pitch_mekf_measurement_relative_deg;
  // V46ab no-control-prediction end""",
        text,
        flags=re.S,
    )

def normalize_config(text: str) -> str:
    text = text.replace(
        "v46ac_light_delay_compensation_20260918",
        "v46ab_no_control_prediction_20260918",
    )
    text = re.sub(
        r"\n// V46ac autonomous timing compensation begin\n.*?\n// V46ac autonomous timing compensation end",
        "",
        text,
        flags=re.S,
    )
    return text
