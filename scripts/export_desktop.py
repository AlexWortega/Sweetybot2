"""Copy the verified v04 ZIP to Desktop without replacing previous deliveries."""
import hashlib,json,shutil
from pathlib import Path
from scripts.common import ROOT,require_mechanical_gate

def main():
 require_mechanical_gate()
 source=ROOT/'dist/Sweetybot2-v04.zip';target=Path.home()/'Desktop'/source.name
 if not source.is_file():raise FileNotFoundError('Run scripts.package_release first')
 temporary=target.with_suffix('.zip.tmp');shutil.copy2(source,temporary)
 digest=hashlib.file_digest(source.open('rb'),'sha256').hexdigest()
 assert hashlib.file_digest(temporary.open('rb'),'sha256').hexdigest()==digest
 temporary.replace(target);target.with_suffix('.zip.sha256').write_text(digest+'  '+target.name+'\n')
 print(json.dumps({'source_folder':str(ROOT),'archive':str(target),'sha256':digest},indent=2))
if __name__=='__main__':main()
