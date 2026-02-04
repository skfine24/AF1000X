SYUBEA AF1000X Firmware Updater

1) 펌웨어 준비
- merged.bin 단일 파일을 준비합니다.
- SYUBEA_Update.exe에서 직접 선택해도 되고, firmware 폴더에 넣어두면 찾기 쉽습니다.

2) 업로드 절차
- 보드를 USB로 연결합니다.
- SYUBEA_Update.exe 실행
- 포트 선택
- 펌웨어 파일 선택
- 업로드

3) 다운로드 모드 진입
- BOOT 버튼을 누른 채 RESET을 눌렀다가 BOOT를 떼면 다운로드 모드로 들어갑니다.
- 연결이 안되면 다시 시도하세요.

4) 빌드 (개발자용)
- build.bat 실행하면 dist\SYUBEA_Update.exe 생성됩니다.

주의
- 보드가 ESP32-S3 내장 USB(USB CDC)로 잡히는지 확인하세요.
- 드라이버가 필요한 보드라면 드라이버를 먼저 설치해야 합니다.

