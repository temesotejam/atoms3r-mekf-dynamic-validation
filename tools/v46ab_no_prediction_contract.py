"""Reverse V46ab's autonomous no-prediction angle changes for retained legacy hashes."""
import re

def normalize_runner(text: str) -> str:
    return re.sub(
        r"\n\s*// V46ab no-control-prediction begin\n.*?\n\s*// V46ab no-control-prediction end",
        "",
        text,
        flags=re.S,
    )

def normalize_config(text: str) -> str:
    return text.replace(
        "v46ab_no_control_prediction_20260918",
        "v46aa_control_upright_zero_20260918",
    )
