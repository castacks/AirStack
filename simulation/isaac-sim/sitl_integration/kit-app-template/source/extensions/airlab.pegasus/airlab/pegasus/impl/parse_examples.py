"""Helper to parse the snippets out of the example file"""

from pathlib import Path


# ==============================================================================================================
def extract_snippets() -> list[tuple[str, str]]:
    """Read the sample file containing the script node snippets in a standard form and parse out the scripts.
    Returns:
        List of (title, script) strings for each snippet in the file.
    Raises:
        IOError if the script node sample file could not be read
    """
    snippets = []
    example_scripts_path = Path(__file__).parent / "ascentnode_example_scripts.py"
    with open(example_scripts_path, "r", encoding="utf-8") as file:
        file_contents = file.read().split("# # # DELIMITER # # #")
        for script in file_contents[1:]:
            script = script.strip()
            script_lines = script.splitlines(keepends=True)
            script_title_line = script_lines[0]
            script_title = script_title_line.strip()[9:-1]
            script_content = "".join(script_lines[1:])
            snippets.append((script_title, script_content))

    return snippets
