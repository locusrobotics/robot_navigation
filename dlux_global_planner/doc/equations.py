#!/usr/bin/env python3
from pathlib import Path
import re
import requests


def clean(s):
    new_s = ''
    for c in s:
        if c in ' \\(){}':
            continue
        elif c.isalnum():
            new_s += c
        else:
            new_s += '_'
    return new_s


EQ_PATTERN = re.compile(r'!eq\[([^\]]+)\]')
doc_folder = Path('doc')
src_files = [p for p in doc_folder.iterdir() if p.suffix == '.md']
existing_gifs = [p for p in doc_folder.iterdir() if p.suffix == '.gif']
gifs_to_remove = set(existing_gifs)

for src_file in src_files:
    print(src_file.stem)
    with open(src_file) as f:
        s = f.read()
    m = EQ_PATTERN.search(s)
    while m:
        equation = m.group(1)
        img_filename = doc_folder / (clean(equation) + '.gif')
        if not img_filename.exists():
            url = 'http://latex.codecogs.com/gif.download?' + equation
            print('\t{} => {}'.format(m.group(0), img_filename))
            img = requests.get(url)
            with open(img_filename, 'wb') as f:
                f.write(img.content)
        if img_filename in gifs_to_remove:
            gifs_to_remove.remove(img_filename)
        s = s.replace(m.group(0), '![%s](%s)' % (equation, img_filename))
        m = EQ_PATTERN.search(s)

    with open(src_file.name, 'w') as f:
        f.write(s)

for gif_to_remove in gifs_to_remove:
    print('Deleting {}'.format(gif_to_remove))
    gif_to_remove.unlink()
