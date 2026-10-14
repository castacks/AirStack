from minio import Minio
from tqdm import tqdm
from minio.error import S3Error
from pathlib import Path


# Do not change these prefilled constant
const_endpoint  = "airlab-share-01.andrew.cmu.edu:9000"
const_bucket    = "macvo-zed-data-icra25"
const_accesskey = "E2T3eWeBfKKoNY1YTJ0p"
const_secretkey = "gxTGRvjK6VfLPzd8n0cK2MFE4Xi3DmnoFHmjPFio"
# End

class TqdmProgress:
    def __init__(self):
        self.pbar = None

    def set_meta(self, object_name: str, total_length: int):
        self.pbar = tqdm(total=total_length,unit="B",unit_scale=True,desc=f"Downloading {object_name}")

    def update(self, length: int):
        if self.pbar: self.pbar.update(length)

def main(download_dst: Path):
    if not download_dst.exists():
        if input(f"{download_dst} does not exists. Want to create this folder? [y/n]").lower().strip() != 'y':
            raise Exception("Aborted by the user.")        
        download_dst.mkdir(parents=True, exist_ok=False)

    client  = Minio(
        endpoint=const_endpoint, access_key=const_accesskey, secret_key=const_secretkey, secure=True
    )
    
    objects = client.list_objects(const_bucket, recursive=True)
    for obj in objects:
        object_name = obj.object_name
        if object_name is None or obj.is_dir: continue
        
        dst = Path(download_dst, object_name)
        
        if not dst.parent.exists(): dst.parent.mkdir(parents=True)
        
        try:
            client.fget_object(const_bucket, object_name, str(dst), progress=TqdmProgress())
        except S3Error as e:
            print(f"Error downloading {obj.object_name}: {e}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(prog="MAC-VO ICRA 2025 Conference-day Zed Dataset Downloader")
    parser.add_argument("--dst", type=Path, help="Destination folder for")
    args   = parser.parse_args()

    main(args.dst)
