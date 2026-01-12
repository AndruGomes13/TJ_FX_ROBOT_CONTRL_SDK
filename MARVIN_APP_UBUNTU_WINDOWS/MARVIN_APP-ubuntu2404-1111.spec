# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['UI_FX2.py'],
    pathex=['python'],
    binaries=[('python/libMarvinSDK.so', '.'), ('python/*.so', '.')],
    datas=[('python/*.py', 'python'), ('src/logo.png', 'src')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='MARVIN_APP-ubuntu2404-1111',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['src/logo.png'],
)
