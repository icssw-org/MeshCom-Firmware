import urllib.request
import json
from pathlib import Path
import os.path

storage_dir = "."



release_url = 'https://api.github.com/repos/icssw-org/MeshCom-Firmware/releases'

def get_releases():
    with urllib.request.urlopen(release_url) as response:
        return json.loads(response.read())
    
def download_asset(assets,filename,target_filename):
    for asset in assets:
        if asset["name"] == filename:
            asset_url = asset["browser_download_url"]
            if os.path.isfile(target_filename):
                    print(target_filename +  " already exists -> skipping")
            else:
                    print(target_filename +  " downloading")
                    urllib.request.urlretrieve(asset_url, target_filename)


def get_target_hardware(asset):

    filename = asset["name"]

    hw_dict = {
    "vision-master-e290.bin": "heltec_e290",
    "heltec_wifi_lora_32_V2.bin": "heltecv2",
    "heltec_wifi_lora_32_V3.bin": "heltecv3",
    "E22-DevKitC.bin": "e22",
    "E22_1262_S3-DevKitC-1-N16R8.bin": "e22-1262-s3",
    "E22_1268_S3-DevKitC-1-N16R8.bin": "e22-1268-s3",
    "ttgo_tbeam.bin": "tbeam",
    "ttgo_tbeam_SX1262.bin" : "tbeamSX1262",
    "ttgo_tbeam_SX1268.bin" : "tbeamSX1268",
    "ttgo-lora32-v21.bin" : "tlora",
    }

    bootloader_dict = {
    "vision-master-e290.bin": "bootloader-s3.bin",
    "heltec_wifi_lora_32_V2.bin": "bootloader.bin",
    "heltec_wifi_lora_32_V3.bin": "bootloader-s3.bin",
    "E22-DevKitC.bin": "bootloader.bin",
    "E22_1262_S3-DevKitC-1-N16R8.bin": "bootloader.bin",
    "E22_1268_S3-DevKitC-1-N16R8.bin": "bootloader.bin",
    "ttgo_tbeam.bin": "bootloader.bin",
    "ttgo_tbeam_SX1262.bin" : "bootloader.bin",
    "ttgo_tbeam_SX1268.bin" : "bootloader.bin",
    "ttgo-lora32-v21.bin" : "bootloader.bin",
    }

    safeboot_dict = {
    "vision-master-e290.bin": "safeboot-s3.bin",
    "heltec_wifi_lora_32_V2.bin": "safeboot.bin",
    "heltec_wifi_lora_32_V3.bin": "safeboot-s3.bin",
    "E22-DevKitC.bin": "safeboot.bin",
    "E22_1262_S3-DevKitC-1-N16R8.bin": "safeboot-s3.bin",
    "E22_1268_S3-DevKitC-1-N16R8.bin": "safeboot-s3.bin",
    "ttgo_tbeam.bin": "safeboot.bin",
    "ttgo_tbeam_SX1262.bin" : "safeboot.bin",
    "ttgo_tbeam_SX1268.bin" : "safeboot.bin",
    "ttgo-lora32-v21.bin" : "safeboot.bin",
    }


    if filename in hw_dict.keys():
        return [hw_dict[filename] , bootloader_dict[filename], safeboot_dict[filename]]
    
    else:
        return [ None, None, None ]



if __name__ == "__main__":
    print("MeshCom Releases Downloader")
    meshcom_fw_path = storage_dir + "/meshcom_fw"
    meshcom_release_path = meshcom_fw_path + "/release"
    meshcom_prerelease_path = meshcom_fw_path + "/prerelease"


    Path(meshcom_release_path).mkdir(parents=True, exist_ok=True)
    Path(meshcom_prerelease_path).mkdir(parents=True, exist_ok=True)

    releases = get_releases()

    for release in releases:
        print("---------------------------")
        tagname = release['tag_name']
        print(tagname)
        if release["prerelease"]:
            target_path = meshcom_prerelease_path + "/" + tagname + "/"
        else:
            target_path = meshcom_release_path + "/" + tagname + "/"

        for asset in release['assets']:
            [target_hw , target_bootloader, target_safeboot] = get_target_hardware(asset)
            filename = asset["name"]
            asset_url = asset["browser_download_url"]

            if target_hw is not None:
                target_asset_path = target_path + target_hw + "/"

                Path(target_asset_path).mkdir(parents=True, exist_ok=True)

                target_filename = target_asset_path + "firmware.bin"
                if os.path.isfile(target_filename):
                    print(tagname + " - " + filename +  " already exists -> skipping")
                else:
                    print(tagname + " - " + filename +  " downloading")
                    urllib.request.urlretrieve(asset_url, target_filename)

                # Download corresponding bootloader + safeboot + partitions + otadata
                download_asset(release['assets'],target_bootloader,target_asset_path + "bootloader.bin")
                download_asset(release['assets'],target_safeboot,target_asset_path + "safeboot.bin")
                download_asset(release['assets'],"partitions.bin",target_asset_path + "partitions.bin")
                download_asset(release['assets'],"otadata.bin",target_asset_path + "otadata.bin")

                                    
