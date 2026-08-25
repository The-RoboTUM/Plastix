#!/usr/bin/env python3
"""
Cross-check the browser UI's three files against each other.

There is no build step between index.html, style.css and app.js, so nothing
catches the mismatches that break a page silently: an id the script looks up
that the markup never defines, a keycap the script never wires, a CSS class the
script toggles that no rule styles. Each of those leaves a page that still
loads and still looks right in a screenshot — and has a dead control on it.

This check found exactly that on the first run: every arrow keycap carried
`data-key="up"` (the name the NODE uses) while the wiring looked keys up in the
browser's `ArrowUp` vocabulary, so clicking or touching the on-screen arrows
did nothing and they never lit up. Keyboard arrows worked, which is why it
would have taken a tablet to notice.

    python3 test/check_web_ui_assets.py
"""
import os
import re
import sys

WEB = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'gripperx_teleop', 'web',
)


def check():
    html = open(os.path.join(WEB, 'index.html')).read()
    js = open(os.path.join(WEB, 'app.js')).read()
    css = open(os.path.join(WEB, 'style.css')).read()
    problems = []

    # 1. Every element the script reaches for must exist in the markup.
    html_ids = set(re.findall(r'\bid="([^"]+)"', html))
    js_ids = (set(re.findall(r"\$\('([^']+)'\)", js))
              | set(re.findall(r"getElementById\('([^']+)'\)", js)))
    for missing in sorted(js_ids - html_ids):
        problems.append(f'app.js looks up id "{missing}", which the HTML never defines')

    # 2. Every drawn keycap must be either a held key or a one-shot action, and
    #    every held key must have a keycap. data-key speaks the NODE's names.
    # Derived from the HOLD_KEYS literal itself rather than a hand-kept list,
    # so a rebinding (2026-08-24 moved the spins to 0 and 9) cannot leave this
    # check quietly validating the previous key set.
    block = re.search(r'const HOLD_KEYS = \{(.*?)\};', js, re.S)
    node_hold = set(re.findall(r":\s*'([\w]+)'", block.group(1))) if block else set()
    caps = re.findall(r'<button class="keycap([^"]*)" data-key="([^"]+)"([^>]*)>', html)
    drawn = {key for _, key, _ in caps}
    for _, key, rest in caps:
        if key not in node_hold and 'data-event=' not in rest:
            problems.append(
                f'keycap data-key="{key}" is neither a held key nor carries a '
                'data-event — clicking it would do nothing')
    for key in sorted(node_hold - drawn):
        problems.append(f'held key "{key}" has no keycap on the page')

    # 3. Classes the script toggles must be styled, or the state is invisible.
    toggled = (set(re.findall(r"classList\.(?:add|toggle)\('([\w-]+)'", js))
               | set(re.findall(r"className = '([\w-]+)'", js)))
    css_classes = set(re.findall(r'\.([a-zA-Z][\w-]*)', css))
    for name in sorted(toggled - css_classes):
        problems.append(f'app.js toggles class "{name}", which style.css never styles')

    # 4. The SVG skeleton the robot view is drawn into.
    for need in ('id="chassis"', 'id="modules"', 'id="twist"', 'class="body"',
                 'class="nose"', 'class="front-label"', 'id="arrowhead"'):
        if need not in html:
            problems.append(f'the robot SVG is missing {need}, which app.js fills in')

    # 5. No external requests: the robot has no internet, and this page must
    #    render identically on a laptop that has none either.
    for external in re.findall(r'(?:src|href)="(https?://[^"]+)"', html):
        problems.append(f'index.html references an external URL: {external}')

    # 6. Crude balance check — cheaper than shipping a JS parser.
    for opener, closer in (('{', '}'), ('(', ')'), ('[', ']')):
        if js.count(opener) != js.count(closer):
            problems.append(f'app.js has unbalanced {opener}{closer}')

    print(f'keycaps: {len(caps)}   ids used by app.js: {len(js_ids)}   '
          f'classes toggled: {len(toggled)}')
    return problems


def main():
    problems = check()
    if problems:
        print('\nFAILED:')
        for problem in problems:
            print('  -', problem)
        return 1
    print('OK — markup, styles and script agree')
    return 0


if __name__ == '__main__':
    sys.exit(main())
