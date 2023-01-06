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

for file in os.listdir(f"{path}/img"):
    os.remove(os.path.join(f"{path}/img/", file))
    #print(os.path.join(f"{path}/img/", file))

print(f"All files in '{path}/img' have been removed.")