import argparse
import hashlib
import io
from pathlib import Path, PurePosixPath
import tempfile
from urllib.request import Request, urlopen
import zipfile

ROOT=Path(__file__).resolve().parents[1]
ASSET_REVISION='runtime-assets-2026.09.28.2'
ASSET_SHA256='d6b9d374e7e523db11b6ea093256e83e1c744c9b3c0a82cbc69042e74f1fbc31'
ASSET_URL='https://github.com/Leo-TY-H/wt-em-calc/releases/download/v2026.09.28.2/runtime-assets.zip'
DIRECTORIES=('references/jet-catalog/','references/prop-mass/','references/prop-propulsion/',
    'references/prop-vehicles/','references/aircraft-modifications/','references/vehicle-names/')
FILES=('references/body-gameparams.blkx','references/body-gameplay.blkx',
    'references/body-flightmodels-config.blkx','references/prop-native-config.json',
    'references/aircraft-exclusions.json','references/data-sync-config.json',
    'references/data-version.json','app/fonts/wt-symbols.ttf')


def selected(name):
    return name in FILES or (name.startswith(DIRECTORIES) and PurePosixPath(name).suffix in ('.json','.blkx'))


def install_archive(content,root,expected_sha=ASSET_SHA256):
    if hashlib.sha256(content).hexdigest()!=expected_sha:
        raise ValueError('Runtime asset archive checksum mismatch')
    root=Path(root).resolve();installed=0
    with zipfile.ZipFile(io.BytesIO(content)) as archive:
        entries=[]
        for entry in archive.infolist():
            path=PurePosixPath(entry.filename)
            if path.is_absolute() or '..' in path.parts or '\\' in entry.filename:
                raise ValueError('Unsafe runtime asset archive path')
            if entry.is_dir() or len(path.parts)<2:continue
            name=PurePosixPath(*path.parts[1:]).as_posix()
            if not selected(name):continue
            target=(root/name).resolve()
            if not target.is_relative_to(root):raise ValueError('Runtime asset path escapes installation')
            if (entry.external_attr>>16)&0o170000==0o120000:raise ValueError('Runtime asset symlink rejected')
            entries.append((entry,target))

        with tempfile.TemporaryDirectory(prefix='wt-runtime-') as temporary:
            staged=[]
            for i,(entry,target) in enumerate(entries):
                if target.exists():continue
                source=Path(temporary)/str(i);source.write_bytes(archive.read(entry))
                staged.append((source,target))
            for source,target in staged:
                target.parent.mkdir(parents=True,exist_ok=True)

                try:
                    with target.open('xb') as stream:stream.write(source.read_bytes())
                    installed+=1
                except FileExistsError:pass
    return installed


def ensure(root=ROOT):
    root=Path(root).resolve();stamp=root/'.data-sync/runtime-assets.version'
    if stamp.exists() and stamp.read_text().strip()==ASSET_REVISION and all((root/p).is_file() for p in FILES):
        return 0
    print('Installing verified runtime assets (kept outside Git)...',flush=True)
    with urlopen(Request(ASSET_URL,headers={'User-Agent':'WT-EM-runtime-setup'}),timeout=90) as response:
        content=response.read()
    installed=install_archive(content,root)
    if not all((root/p).is_file() for p in FILES):raise ValueError('Required runtime assets missing')
    stamp.parent.mkdir(parents=True,exist_ok=True);stamp.write_text(ASSET_REVISION+'\n',encoding='utf-8')
    print(f'Runtime assets ready; installed {installed} missing files.',flush=True)
    return installed


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    ensure(parser.parse_args().root)
