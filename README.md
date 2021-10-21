# GilBot2021-AutoPilot-Driver-v1

![Gilbot Driver](https://user-images.githubusercontent.com/13266481/138188402-4b2a33d4-84f6-406c-8f56-cd2692e12103.PNG)

전동스크터에 부착한 묘듈과 Nvidia Jetson 보드의 UART 통신을 위한 펌웨어 프로그램입니다.


UART(Universal Asynchronous Receiver Transmitter)란?

- 하나의 선로에 데이터를 서로 정해진 시간차의 동기를 주어 인식하는 통신방식
- 통신 드라이버에 따라 1:1 또는 1:n, n:n의 통신이 가능
- 기본 8비트 데이터 통신으로 9600 baudrate 이용
- (시작1비트)(데이터8비트)(패리티n비트)(종료1비트)



JETSON TX2 Serial 사용 준비
@ sudo systemctl stop nvgetty

@ sudo systemctl disable nvgetty

@ sudo udevadm trigger

@ sudo apt-get install python3-serial

JETSON TX2 Serial 권한 설정

@ ls -al /dev/ttyTHS* # 연결된 THS장치들을 나열

@ id # 현재 로그온한 계정의 권한을 확인

@ sudo usermod -a -G dialout nvidia # nvidia 계정에 dialout 권한을 부여
