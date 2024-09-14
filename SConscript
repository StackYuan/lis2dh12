
from building import *
import rtconfig

cwd = GetCurrentDir()

src = []

src += Glob('lis2dh12.c')
src += Glob('lis2dh12_reg.c')

CPPPATH = [cwd]

if GetDepend('PKG_LIS2DH12_USING_SENSOR_V1'):
    src += ['st_lis2dh12_sensor_v1.c']

group = DefineGroup('lis2dh12_acce', src, depend = ['PKG_USING_LIS2DH12'], CPPPATH = CPPPATH)

Return('group')
