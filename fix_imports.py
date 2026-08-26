import os
import re


def fix_file(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    # import config
    content = re.sub(
        r"^([ \t]*)import config(\r?\n|$)",
        r"\1from path_planning import config\2",
        content,
        flags=re.MULTILINE,
    )

    # import core... -> from path_planning.core import...
    content = re.sub(
        r"^([ \t]*)import core\.",
        r"\1from path_planning.core import ",
        content,
        flags=re.MULTILINE,
    )

    # from core... -> from path_planning.core...
    content = re.sub(
        r"^([ \t]*)from core\.",
        r"\1from path_planning.core.",
        content,
        flags=re.MULTILINE,
    )
    content = re.sub(
        r"^([ \t]*)from core ",
        r"\1from path_planning.core ",
        content,
        flags=re.MULTILINE,
    )

    # import render... -> from path_planning import render / from path_planning.render import...
    content = re.sub(
        r"^([ \t]*)import render\.",
        r"\1from path_planning.render import ",
        content,
        flags=re.MULTILINE,
    )
    content = re.sub(
        r"^([ \t]*)import render(\r?\n|$)",
        r"\1from path_planning import render\2",
        content,
        flags=re.MULTILINE,
    )

    # from render... -> from path_planning.render...
    content = re.sub(
        r"^([ \t]*)from render\.",
        r"\1from path_planning.render.",
        content,
        flags=re.MULTILINE,
    )
    content = re.sub(
        r"^([ \t]*)from render ",
        r"\1from path_planning.render ",
        content,
        flags=re.MULTILINE,
    )

    # from logger_config... -> from path_planning.logger_config...
    content = re.sub(
        r"^([ \t]*)from logger_config",
        r"\1from path_planning.logger_config",
        content,
        flags=re.MULTILINE,
    )

    # from vtx_service... -> from service.vtx_service...
    content = re.sub(
        r"^([ \t]*)from vtx_service\.",
        r"\1from service.vtx_service.",
        content,
        flags=re.MULTILINE,
    )
    content = re.sub(
        r"^([ \t]*)from vtx_service ",
        r"\1from service.vtx_service ",
        content,
        flags=re.MULTILINE,
    )

    # import vtx_service... -> from service import vtx_service...
    content = re.sub(
        r"^([ \t]*)import vtx_service\.",
        r"\1from service import vtx_service.",
        content,
        flags=re.MULTILINE,
    )
    content = re.sub(
        r"^([ \t]*)import vtx_service(\r?\n|$)",
        r"\1from service import vtx_service\2",
        content,
        flags=re.MULTILINE,
    )

    # specific fix for invalid syntax: from service import vtx_service.X
    content = re.sub(
        r"^([ \t]*)from service import vtx_service\.(.*)",
        r"\1from service.vtx_service import \2",
        content,
        flags=re.MULTILINE,
    )

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)


for root, _, files in os.walk("."):
    if ".git" in root or ".venv" in root or "__pycache__" in root:
        continue
    for file in files:
        if file.endswith(".py"):
            fix_file(os.path.join(root, file))
