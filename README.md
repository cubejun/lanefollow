# lanefollow
로봇의 카메라를 이용해 트랙의 라인을 검출한다. 

gstreamer를 이용해 젯슨나노 보드에서 카메라 영상을 읽고, pc에 영상을 전송한다. 

관심영역을 지정하고, 평균 밝기를 지정해 밝기를 보정한 다음 영상을 이진화한다.

이진화 된 영상을 레이블링을 통해 영상속 검출되는 객체들의 개수와 무게중심값에 대한 정보를 확인 할 수 있다.

로봇의 초기 위치는 항상 트랙 중심에 있고 영상의 중심에 있게 정의한다.

![image](https://github.com/cubejun/lanefollow/assets/133946040/81399cf1-e3b4-473e-a60c-ee85f0574bee)


로봇이 트랙의 중앙에 위치하면 라인은 영상의 왼쪽과 오른쪽에 위치하게 된다. 라인검출을 위한 초기값을 영상의 왼쪽과 오른쪽의 임의의 좌표로 설정해둔다.

이전 라인 무게중심과 현재 검출된 객체들의 무게중심의 오차가 가장 작은 객체를 최소값 알고리즘을 통해 찾아낸다.

찾아낸 왼쪽과 오른쪽 라인의 무게중심의 중간좌표를 영상의 중간좌표에 빼는 것으로 라인이 로봇 정면에서 벗어난 정도(error)를 계산했다.

라인이 영상 밖으로 나가는 경우 최소값 알고리즘을 통해 검출되는 객체는 따라가던 라인이 아닌 잡음이나 다른 라인이 검출되는데, 이전 라인 무게중심과 현재 검출된 객체의 오차가 특정 값 이상이 되면 이전 라인 무게중심을 현재 라인의 무게중심으로 업데이트 되도록 만들었다.

![image](https://github.com/cubejun/lanefollow/assets/133946040/a088b4cd-d903-495c-a6b2-42ac120cdb92)



이렇게 검출된 라인의 무게중심을 영상의 중심좌표(로봇의 중심)에 빼는 것으로 라인이 로봇 정면에서 벗어난 정도(error)를 계산했다.
좌측속도명령 = 직진속도 - 게인*error
우측속도명령 = 직진속도 + 게인*error

s버튼을 누르면 모터가 가동되고, ctrl+c를 누르면 코드가 종료된다. 

# robot view, 영상처리결과, 콘솔출력결과


https://github.com/cubejun/lanefollow/assets/133946040/75fc2e7e-261c-4867-bf4e-0b9a81064663




https://github.com/cubejun/lanefollow/assets/133946040/295c9053-b77f-4e89-a184-8d3277ae9708


# human view



https://github.com/cubejun/lanefollow/assets/133946040/1db109ee-6c1c-458f-864c-c9083b0d018e





https://github.com/cubejun/lanefollow/assets/133946040/2695d56e-d93b-4d7f-bd0c-84f5953151cd



