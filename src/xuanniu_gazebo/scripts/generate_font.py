#!/usr/bin/env python3
"""Optional asset-build tool: requires fontTools, never needed by the runtime."""
import argparse
from pathlib import Path
from fontTools import subset
from fontTools.ttLib import TTFont

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('source',help='NotoSansCJK-Regular.ttc')
args=parser.parse_args()
font=TTFont(args.source,fontNumber=2)
assert font['name'].getDebugName(1)=='Noto Sans CJK SC'
text=''.join(p.read_text() for p in (root/'src/xuanniu_gazebo').glob('*.py'))
text+=''.join(chr(i) for i in range(32,127))+'°−手动停自动'
options=subset.Options(); options.name_IDs=['*']; options.name_legacy=True
subsetter=subset.Subsetter(options=options); subsetter.populate(text=text); subsetter.subset(font)
# Use a distinct family for the modified subset and retain copyright metadata.
for name in font['name'].names:
    if name.nameID in (1,3,4,6,16):
        value='SelectorCJK-Regular' if name.nameID in (3,6) else 'Selector CJK'
        name.string=value.encode(name.getEncoding())
font.save(root/'fonts/SelectorCJK.otf')
print('Bundled glyph subset:',root/'fonts/SelectorCJK.otf')
