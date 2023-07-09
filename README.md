<!-- 
Widget https://github.com/anuraghazra/github-readme-stats
Badges https://github.com/Ileriayo/markdown-badges
Icons  https://gist.github.com/rxaviers/7360908
 -->

<h1 align="center">PARK JAE WAN _ STM32_SMART_FARM🐾</h1>

<!-- LANGUAGES Widget -->
<!-- ![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=nrmhvr&exclude_repo=github-readme-stats&hide=ANTLR,Jasmin&langs_count=6&layout=compact&hide_border=true&theme=gruvbox_light) -->
<!--
<img align="left" src="https://github-readme-stats.vercel.app/api?username=nrmhvr&theme=gruvbox_light&hide_border=true&count_private=true&show_icons=false&custom_title=GitHub%20Stats😊"/>
-->

<h2>💻 Working with</h2>
<img src="https://img.shields.io/badge/C++-cppplus-blue?logo=cplusplus&logoColor=white">
<img src="https://img.shields.io/badge/C-C-blue?logo=c&logoColor=white">
<img src="https://img.shields.io/badge/stmicroelectronics-stmicroelectronics-blue?logo=stmicroelectronics&logoColor=#03234B">
<img src="https://img.shields.io/badge/github-github-blue?logo=github&logoColor=#181717">

<!--<h2>📚 Tech Stack</h2> -->
<!-- Languages 
![Java](https://img.shields.io/badge/java-%23ED8B00.svg?style=for-the-badge&logo=java&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54) ![CSS3](https://img.shields.io/badge/css3-%231572B6.svg?style=for-the-badge&logo=css3&logoColor=white) ![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
-->

<!-- Server
![Apache](https://img.shields.io/badge/apache-%23D42029.svg?style=for-the-badge&logo=apache&logoColor=white)
-->
<!-- DB 
![MySQL](https://img.shields.io/badge/mysql-%2300f.svg?style=for-the-badge&logo=mysql&logoColor=white) 
-->
<!-- Frameworks, Platforms and Libraries -->
<!-- ![Vue.js](https://img.shields.io/badge/vuejs-%2335495e.svg?style=for-the-badge&logo=vuedotjs&logoColor=%234FC08D) ![Anaconda](https://img.shields.io/badge/Anaconda-%2344A833.svg?style=for-the-badge&logo=anaconda&logoColor=white) ![Flask](https://img.shields.io/badge/flask-%23000.svg?style=for-the-badge&logo=flask&logoColor=white)  -->

<!-- IDE & Editors -->
<!-- ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white) ![Atom](https://img.shields.io/badge/Atom-%2366595C.svg?style=for-the-badge&logo=atom&logoColor=white) ![Eclipse](https://img.shields.io/badge/Eclipse-FE7A16.svg?style=for-the-badge&logo=Eclipse&logoColor=white) ![Jupyter Notebook](https://img.shields.io/badge/jupyter-%23FA0F00.svg?style=for-the-badge&logo=jupyter&logoColor=white) -->

<!-- <h2>✏️ learning</h2> -->
<!-- <h2>💡 📁 Projects</h2> -->

## 기능
![회로도](image/%EA%B7%B8%EB%A6%BC2.jpg)
- LED 제어: WS2812 LED를 사용하여 다양한 색상의 조명 효과를 생성합니다.
- 온습도 및 조도 측정: DHT11 센서를 사용하여 주변 환경의 온도, 습도 및 조도를 측정합니다.
- ADC 데이터 수집: 외부 센서로부터 아날로그 데이터를 수집합니다.
- 시리얼 통신: UART를 사용하여 Nextion 디스플레이와 통신합니다.(위의 언급 된 기능들의 모니터링 및 제어는 디스플레이를 통해서 가능 합니다.)
- 타이머 인터럽트: 내부 타이머를 사용하여 정기적인 작업을 수행합니다.

## 사용 방법

아래의 단계를 따라 AirMouse를 설정하고 사용할 수 있습니다:

1. 필요한 하드웨어를 구성합니다:
   - STM32F103
   - 릴레이
   - USB 케이블
   - WS2812B RING
   - DHT-11
   - NEXTION DISPLAY
   - soil moisture sensor (토양 수분 센서)

![회로도](image/%ED%9A%8C%EB%A1%9C%EB%8F%84.png)


2. STM32 마이크로컨트롤러에 프로그램을 업로드합니다.

3. LED, 온습도, 조도, ADC 등의 기능을 사용하려면 해당 센서와 하드웨어를 연결합니다.

4. 프로그램 실행 시 LED 조명 효과 및 센서 데이터가 표시됩니다.

5. 동작에 대한 결과는 

[![영상 설명](https://img.youtube.com/vi/qn7wa19mgks/0.jpg)](https://www.youtube.com/watch?v=qn7wa19mgks)


 다음 영상과 비교할 수 있습니다.


## 기여하기

이 프로젝트에 기여하고 싶다면, 다음 단계를 따라주세요:

1. 이 리포지토리를 포크합니다.

2. 개선하거나 수정하고자 하는 기능을 위한 새로운 브랜치를 생성합니다.

3. 변경 사항을 커밋하고, 새로운 브랜치에 푸시합니다.

4. Pull Request를 작성하여 변경 사항을 원본 리포지토리로 제출합니다.

5. 리뷰 및 피드백을 통해 변경 사항을 검토합니다.

## 라이선스

이 프로젝트는 MIT 라이선스에 따라 배포됩니다. 자세한 내용은 `LICENSE` 파일을 참조하세요.