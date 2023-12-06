#include "opencv2/opencv.hpp"
#include "dxl.hpp"
#include <iostream>
#include <queue>
#include <ctime>
#include <unistd.h>
#include <signal.h>
using namespace cv;
using namespace std;
bool ctrl_c_pressed;//시그널 인터럽트
void ctrlc(int)
{
	ctrl_c_pressed = true;
}
int main()//메인함수 시작
{
	string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";//카메라 영상 입력받음
    VideoCapture source(src, CAP_GSTREAMER);
	// VideoCapture source("lanefollow_100rpm_cw.mp4");
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }//키메라 에러처리


	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.156 port=8001 sync=false";
	VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
	if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.156 port=8002 sync=false";
	VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);
	if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	Dxl mx;
	struct timeval start, end1;
	double diff1;
	int lvel = 0, rvel = 0, err;

	Point cameracentroids(320, 45);//영상의 중심좌표

	Mat frame, ROI, gray, bin, color, meangray;//매트변수

	Mat labels, stats, centroids;//레이블링 변수

	bool spress = 0;//s눌림 확인 변수

	//왼쪽 라인 변수
	int cntmin = 0, cntymin = 0, nearx = 0, neary = 0;
	int cx = 0, cy = 0, cw = 0, ch = 0;
	double centermin = 0, centerqbacksave, centerysave = 45, centerymin = 0;
	queue<double> centerq, centeryq;
	centerq.push(200);
	centeryq.push(45);
	//오른쪽 라인 변수
	int cntmin2 = 0, cntymin2 = 0, nearx2 = 0, neary2 = 0;
	int cx2 = 0, cy2 = 0, cw2 = 0, ch2 = 0;
	double centermin2 = 0, err2, centerqbacksave2, centerysave2 = 45, centerymin2 = 0;
	queue<double> centerq2, centeryq2;
	centerq2.push(450);
	centeryq2.push(45);

	double gain = 0.36;
	//속도 100 게인 0.2, 0.18
	//속도 200 게인 0.4 0.36

	signal(SIGINT, ctrlc);              //시그널 핸들러 지정
	if (!mx.open()) { cout << "dxl open error" << endl; return -1; }//다이나믹셀 에러처리
	while (true) {//무한 반복문 시작
		gettimeofday(&start, NULL);//코드 실행 시간측정
		if (mx.kbhit())//키보드입력처리
		{
			char c = mx.getch();//키보드 입력 받은 값 저장
			switch (c)
			{
			case 's': spress = 1; break;//s가 입력되면 다이나믹셀 실행
			}
		}
		if (ctrl_c_pressed) break;//ctrlc 입력되면 코드 종료

		
		source >> frame;//영상을 mat객체에 입력받음
		if (frame.empty()) { cerr << "frame empty!" << endl; break; }//프레임 에러 처리
		ROI = frame(Rect(0, 270, 640, 90));//관심영역 처리
		cvtColor(ROI, gray, COLOR_BGR2GRAY);//컬러 영상 흑백 영상으로 변환
		meangray = gray + (Scalar(100) - mean(gray));//영상 평균값 처리
		threshold(meangray, bin, 130, 255, THRESH_BINARY); //영상 이진화
		//라인 160
		//레인 불 켬 120, 130
		int cnt = connectedComponentsWithStats(bin, labels, stats, centroids);//이진화 영상 레이블링
		cvtColor(bin, color, COLOR_GRAY2BGR);//이진화 영상 컬려영상으로 변환

		centermin = abs(centerq.back() - centroids.at<double>(1, 0));//왼쪽라인 최소값 알고리즘
		centerymin = abs(centeryq.back() - centroids.at<double>(1, 1));

		centermin2 = abs(centerq2.back() - centroids.at<double>(1, 0));//오른쪽 라인 최소값 알고리즘
		centerymin2 = abs(centeryq2.back() - centroids.at<double>(1, 1));

		for (int i = 1; i < cnt; i++) {//레이블링으로 검출된 객체 개수만큼 반복
			int* p = stats.ptr<int>(i);//스탯 내용 포인터 변수에 저장
			double* c = centroids.ptr<double>(i);//무게중심 내용 포인터 변수에 저장

			if (p[4] > 100) {//객체의 크기가 100 이상일때

				rectangle(color, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);//검출된 객체에 파란색 박스 그림
				circle(color, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2);//검출된 객체의 무게중심에 파란색 원 그림

				if ((centermin >= abs(centerq.back() - c[0])) || (centerymin >= abs(centeryq.back() - c[1]))) {//왼쪽 라인과 검출된 객체의 차이 중 가장 가까운 객체 검출
					if ((abs(centerq.back() - c[0]) < 150) && (abs(centeryq.back() - c[1]) < 50)) {//이전의 라인과 현재 검출된 객체의 무게중심이 특정값 이하일때만 동작
						cntmin = i;//이전 라인과 현재 검출된 객체가 가장 가까울때의 인덱스 저장
						centermin = abs(centerq.back() - c[0]);//최소값 알고리즘
						centerymin = abs(centeryq.back() - c[1]);//최소값 알고리즘
						nearx = centroids.at<double>(cntmin, 0);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 x값 저장
						neary = centroids.at<double>(cntmin, 1);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 y값 저장
						cx = stats.at<int>(cntmin, 0);
						cy = stats.at<int>(cntmin, 1);
						cw = stats.at<int>(cntmin, 2);
						ch = stats.at<int>(cntmin, 3);//이전 라인의 무게중심과 가장 가까운 객체의 박스 좌표 정보 저장
					}
				}

				if ((centermin2 >= abs(centerq2.back() - c[0])) || (centerymin2 >= abs(centeryq2.back() - c[1]))) {//오른쪽 라인과 검출된 객체의 차이 중 가장 가까운 객체 검출
					if ((abs(centerq2.back() - c[0]) < 150) && (abs(centeryq2.back() - c[1]) < 50)) {//이전의 라인과 현재 검출된 객체의 무게중심이 특정값 이하일때만 동작
						cntmin2 = i;//이전 라인과 현재 검출된 객체가 가장 가까울때의 인덱스 저장
						centermin2 = abs(centerq2.back() - c[0]);//최소값 알고리즘
						centerymin2 = abs(centeryq2.back() - c[1]);//최소값 알고리즘
						nearx2 = centroids.at<double>(cntmin2, 0);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 x값 저장
						neary2 = centroids.at<double>(cntmin2, 1);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 y값 저장
						cx2 = stats.at<int>(cntmin2, 0);
						cy2 = stats.at<int>(cntmin2, 1);
						cw2 = stats.at<int>(cntmin2, 2);
						ch2 = stats.at<int>(cntmin2, 3);//이전 라인의 무게중심과 가장 가까운 객체의 박스 좌표 정보 저장
					}
				}
			}
		}
		//1
		centerq.push(nearx);//검출된 왼쪽라인의 무게중심 x값 푸쉬
		centeryq.push(neary);//검출된 왼쪽라인의 무게중심 y값 푸쉬
		if (abs(centerq.back() - centerq.front()) > 200) {//이전의 라인과 현재 검출된 라인의 무게중심 x좌표 차이가 특정 값 이상이면
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
			cx = 0; cy = 0; cw = 0; ch = 0;
			//이전의 라인 무게중심값 저장
		}
		if (abs(centeryq.back() - centeryq.front()) > 60) {//이전의 라인과 현재 검출된 라인의 무게중심 y좌표 차이가 특정 값 이상이면
			centeryq.push(centeryq.front());
			centeryq.pop();
			centeryq.push(centeryq.back());
			centeryq.pop();
			//이전의 라인 무게중심값 저장
		}
		centerqbacksave = centerq.back();//현재 검출된 왼쪽 라인의 좌표를 변수에 저장
		centerysave = centeryq.back();//현재 검출된 왼쪽 라인의 좌표를 변수에 저장

		circle(color, Point(centerqbacksave, centerysave), 2, Scalar(0, 0, 255), 2);//검출된 왼쪽 라인의 무게중심에 빨간색 원 그림
		rectangle(color, Rect(cx, cy, cw, ch), Scalar(0, 0, 255), 2);//검출된 왼쪽 라인에 빨간색 사각형 그림
		//2
		centerq2.push(nearx2);//검출된 오른쪽라인의 무게중심 x값 푸쉬
		centeryq2.push(neary2);//검출된 오른쪽라인의 무게중심 y값 푸쉬
		if (abs(centerq2.back() - centerq2.front()) > 200) {//이전의 라인과 현재 검출된 라인의 무게중심 x좌표 차이가 특정 값 이상이면
			centerq2.push(centerq2.front());
			centerq2.pop();
			centerq2.push(centerq2.back());
			centerq2.pop();
			cx2 = 0; cy2 = 0; cw2 = 0; ch2 = 0;
		}
		if (abs(centeryq2.back() - centeryq2.front()) > 60) {//이전의 라인과 현재 검출된 라인의 무게중심 y좌표 차이가 특정 값 이상이면
			centeryq2.push(centeryq2.front());
			centeryq2.pop();
			centeryq2.push(centeryq2.back());
			centeryq2.pop();
		}
		centerqbacksave2 = centerq2.back();//현재 검출된 오른쪽 라인의 좌표를 변수에 저장
		centerysave2 = centeryq2.back();//현재 검출된 오른쪽 라인의 좌표를 변수에 저장
		circle(color, Point(centerqbacksave2, centerysave2), 2, Scalar(0, 0, 255), 2);//검출된 오른쪽 라인의 무게중심에 빨간색 원 그림
		rectangle(color, Rect(cx2, cy2, cw2, ch2), Scalar(0, 0, 255), 2);//검출된 오른쪽 라인에 빨간색 사각형 그림

		line(color, Point(centerq.back(), centeryq.back()), Point(centerq2.back(), centeryq2.back()), Scalar(255, 0, 255), 2);//왼쪽과 오른쪽 라인의 무게중심을 잇는 선 그림
		line(color, Point(320, 0), Point(320, 90), Scalar(0, 255, 255), 2);//영상의x좌표 중심에 수직축 그림
		if (centerysave2 >= centerysave)
			circle(color, Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave2 - centerysave) / 2 + centerysave), 2, Scalar(255, 255, 255), 2);
		else circle(color, Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave - centerysave2) / 2 + centerysave2), 2, Scalar(255, 255, 255), 2);
		//왼쪽 라인과 오른쪽 라인의 무게중심의 중간에 원 그림
		
		err = cameracentroids.x - ((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave);//왼쪽 라인의 무게중심과 오른쪽 라인의 무게중심의 중간값을 영상의 중간좌표에 뺀 값을 에러에 저장
		lvel = 200 - gain * err;//왼쪽 바퀴 속도
		rvel = -(200 + gain * err);//오른쪽 바퀴 속도

		
		//1
		centerq.pop();//이전 왼쪽 라인 x좌표값 제거
		centeryq.pop();//이전 왼쪽 라인 y좌표값 제거

		//2
		centerq2.pop();//이전 오른쪽 라인 x좌표값 제거
		centeryq2.pop();//이전 오른쪽 라인 x좌표값 제거

		writer1 << frame;//영상 출력
		writer2 << color;//영상처리한 영상 출력
		if (spress)//s버튼이 입력되면 모터 구동
		{
			if (!mx.setVelocity(lvel, rvel)) { cout << "setVelocity error" << endl; return -1; }//모터구동에러처리
		}
		usleep(20*1000);//20ms 쉼
		
		
		gettimeofday(&end1, NULL);//코드 동작 시간 측정
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;//코드 동작 시간 변수에 저장
		cout << "err : " << err;//에러값 출력
		cout << ", lvel : " << lvel;//왼쪽 모터 속도 출력
		cout << ", rvel :" << rvel;//오른쪽 모터 속도 출력
		cout << ", time : " << diff1 << endl;//코드 동작시간 출력
	}
    mx.close();//다이나믹셀 종료
	return 0;//0 반환
}