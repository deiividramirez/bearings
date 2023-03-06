"""
This script removes all files in the 'img' folder.
Due to the fact that the 'img' folder is tracked by git,
this script is necessary to avoid the upload of unnecessary files.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import os 

from pathlib import Path
path = Path(__file__).parent.absolute()

imgs = os.listdir(f"{path}/img")
nottodelete = 0

for file in imgs:
    if (file.split("_")[1].split(".")[0]) != "1":
        os.remove(os.path.join(f"{path}/img/", file))
    else:
        nottodelete += 1

print(f"{len(imgs)-nottodelete} files have been removed from '{path}/img'.")