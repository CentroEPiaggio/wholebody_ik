import re
import numpy as np

out=open('parsed.log','w')
i=0
mat = np.array([0.0, 0.0, 0.0])
vec = np.array([0.0, 0.0, 0.0])
with open('wholebody_ik_test.log') as f:
    for line in f:
        if 'Elapsed time' in line:
            if i>2:
                mat = np.concatenate(mat,vec)
                i=0
            if line.find("Elapsed time")!=-1:
                out.write(line)

out.close()                
