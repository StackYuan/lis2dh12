
from building import *
import rtconfig

cwd = GetCurrentDir()

src = []

src += Glob('*.c')
CPPPATH = [cwd]

group = DefineGroup('lis2dh12_acce', src, depend = ['PKG_USING_LIS2DH12'], CPPPATH = CPPPATH)

Return('group')
