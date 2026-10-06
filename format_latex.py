import re

with open('README.md', 'r') as f:
    text = f.read()

# Replace inline block math: $$ math $$ with:
# $$
# math
# $$
# But also ensure there is a blank line before and after the $$ block.

def replacer(match):
    math_content = match.group(1).strip()
    return f"\n\n$$\n{math_content}\n$$\n\n"

# The regex matches $$ ... $$ non-greedily
new_text = re.sub(r'\$\$(.*?)\$\$', replacer, text, flags=re.DOTALL)

# Cleanup multiple newlines
new_text = re.sub(r'\n{3,}', '\n\n', new_text)

with open('README.md', 'w') as f:
    f.write(new_text)

