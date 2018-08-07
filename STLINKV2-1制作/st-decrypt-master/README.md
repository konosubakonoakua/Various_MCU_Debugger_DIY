# st-decrypt
Utility for encrypting/decrypting ST-Link firmware images.
It is mostly based on decompiled code, which is not human-readable (and probably never was in the first place).
Compiled archive can be found in `dist` folder.

## Usage
Decrypt binary:
```
$ java -jar st_decrypt.jar --key "best performance" -i firmware.bin  -o firmware_decrypted.bin
```
Encrypt binary:
```
$ java -jar st_decrypt.jar --key "encryption key" -i firmware.bin  -o firmware_encrypted.bin --encrypt
```
