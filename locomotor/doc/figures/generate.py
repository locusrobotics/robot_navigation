import os
import requests
import re
import urllib

WSD_URL = 'http://www.websequencediagrams.com/'

def getSequenceDiagram(text, out_fn, style='modern-blue'):
    request = {}
    request["message"] = text
    request["style"] = style
    request["apiVersion"] = "1"

    r = requests.post(WSD_URL, data=request)
    response = r.json()
    if 'img' not in response:
        print("Invalid response from server.")
        return

    with open(out_fn, 'w') as f:
        print out_fn
        r = requests.get(WSD_URL + response['img'])
        f.write(r.content)


if __name__ == '__main__':
    current_directory = os.path.abspath(os.path.dirname(__file__))
    for fn in os.listdir(current_directory):
        base, ext = os.path.splitext(fn)
        if ext != '.flow':
            continue
        text = open(os.path.join(current_directory, fn)).read()
        out_fn = os.path.join(current_directory, base + '.png')
        getSequenceDiagram(text, out_fn)
