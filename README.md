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






i2c_transmit(ai_mode, drv_mode, drv_throttle, lamp_act, signal_lamp, horn_act)

(0~3)	ai_mode: 0=자동화 사용 안함, 1=사용자 허가하에 사용, 2=강제사용, 3=없음

(0~3)	drv_mode: 0=리니어브레이크, 1=전진모드, 2=후진모드, 3=급브레이크

(0~127)	drv_throttle: drv_mode1과 2에만 반응하는 속도레벨

(0~1)	lamp_act: 0=램프 끄기, 1=램프 켜기

(0~3)	signal_lamp: 0=방향등X, 1=좌측방향등, 2=우측방향등, 3=비상등

(0~1)	horn_act: 0=크락션끄기, 1=크락션켜기
