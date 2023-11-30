#include "opencv2/opencv.hpp"
#include "dxl.hpp"
#include <iostream>
#include <queue>
#include <ctime>
#include <unistd.h>
#include <signal.h>
using namespace cv;
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}
int main()
{
	string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    VideoCapture source(src, CAP_GSTREAMER);
	// VideoCapture source("lanefollow_100rpm_cw.mp4");
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }


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
	int goal1 = 0, goal2 = 0;

	Point cameracentroids(320, 45);//영상의 중심좌표

	Mat frame, ROI, gray, bin, color, meangray;//매트변수

	Mat labels, stats, centroids;//레이블링 변수

	bool spress = 0;

	//왼쪽
	int cntmin = 0, cntymin = 0, nearx = 0, neary = 0;
	int cx = 0, cy = 0, cw = 0, ch = 0;
	double centermin = 0, centerqbacksave, centerysave = 45, centerymin = 0;
	queue<double> centerq, centeryq;
	centerq.push(200);
	centeryq.push(45);
	//오른쪽 변수
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
	if (!mx.open()) { cout << "dxl open error" << endl; return -1; }
	while (true) {
		gettimeofday(&start, NULL);
		if (mx.kbhit())
		{
			char c = mx.getch();
			switch (c)
			{
			case 's': spress = 1; break;
			}
		}
		if (ctrl_c_pressed) break;

		
		source >> frame;
		if (frame.empty()) { cerr << "frame empty!" << endl; break; }
		ROI = frame(Rect(0, 270, 640, 90));
		cvtColor(ROI, gray, COLOR_BGR2GRAY);
		meangray = gray + (Scalar(100) - mean(gray));
		threshold(meangray, bin, 130, 255, THRESH_BINARY);        //adaptiveThreshold(meangray, adbin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 301, 10);
		//라인 160
		//레인 불 켬 120, 130
		int cnt = connectedComponentsWithStats(bin, labels, stats, centroids);
		cvtColor(bin, color, COLOR_GRAY2BGR);

		/*centermin = 320 - centroids.at<double>(1, 0);
		if (centermin < 0)centermin = -centermin;*/

		centermin = abs(centerq.back() - centroids.at<double>(1, 0));
		centerymin = abs(centeryq.back() - centroids.at<double>(1, 1));

		centermin2 = abs(centerq2.back() - centroids.at<double>(1, 0));
		centerymin2 = abs(centeryq2.back() - centroids.at<double>(1, 1));
		//err = centroids.at<double>(1, 0) - cameracentroids.x;

		/*else if (cnt <= 1) {
			centermin = abs(centerq.front() - centerq.back());
			err = centerq.back() - cameracentroids.x;
		}*/
		//추적하던라인이 없어졌을때를 대비하는 코드인데 실패함

		//pastpre = cameracentroids.x - centroids.at<double>(1, 1);

		for (int i = 1; i < cnt; i++) {
			int* p = stats.ptr<int>(i);
			//double* cerr = centroids.ptr<double>(i+1);
			double* c = centroids.ptr<double>(i);

			if (p[4] > 100) {
				/*if (err >= 0) {
					if (err > 320 - c[0])errmin = c[0] - 320;
				}
				else if (err < 0) {
					if (err < 320 - c[0])errmin = c[0] - 320;
				}*/

				rectangle(color, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);
				circle(color, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2);

				//circle(color, Point(c[0], c[1]), 2, Scalar(0, 0, 255), 2);


				//if (centermin >= abs(centerq.back() - c[0])) {
				//	cntmin = i;
				//	centermin = abs(centerq.back() - c[0]);
				//	nearx = centroids.at<double>(cntmin, 0);
				//	neary = centroids.at<double>(cntmin, 1);
				//	cx = stats.at<int>(cntmin, 0);
				//	cy = stats.at<int>(cntmin, 1);
				//	cw = stats.at<int>(cntmin, 2);
				//	ch = stats.at<int>(cntmin, 3);
				//	//cout << nearx << endl;
				//}
				//if (centerymin >= abs(centeryq.back() - c[1])) {
				//	cntmin = i;
				//	centerymin = abs(centeryq.back() - c[1]);
				//	nearx = centroids.at<double>(cntmin, 0);
				//	neary = centroids.at<double>(cntmin, 1);
				//	cx = stats.at<int>(cntmin, 0);
				//	cy = stats.at<int>(cntmin, 1);
				//	cw = stats.at<int>(cntmin, 2);
				//	ch = stats.at<int>(cntmin, 3);
				//	//cout << nearx << endl;
				//}

				if ((centermin >= abs(centerq.back() - c[0])) || (centerymin >= abs(centeryq.back() - c[1]))) {
					if ((abs(centerq.back() - c[0]) < 150) && (abs(centeryq.back() - c[1]) < 50)) {
						cntmin = i;
						centermin = abs(centerq.back() - c[0]);
						centerymin = abs(centeryq.back() - c[1]);
						nearx = centroids.at<double>(cntmin, 0);
						neary = centroids.at<double>(cntmin, 1);
						cx = stats.at<int>(cntmin, 0);
						cy = stats.at<int>(cntmin, 1);
						cw = stats.at<int>(cntmin, 2);
						ch = stats.at<int>(cntmin, 3);
						//cout << abs(centerq.back() - c[0]) << endl;
						//cout << nearx << endl;
					}
				}

				if ((centermin2 >= abs(centerq2.back() - c[0])) || (centerymin2 >= abs(centeryq2.back() - c[1]))) {
					if ((abs(centerq2.back() - c[0]) < 150) && (abs(centeryq2.back() - c[1]) < 50)) {
						cntmin2 = i;
						centermin2 = abs(centerq2.back() - c[0]);
						centerymin2 = abs(centeryq2.back() - c[1]);
						nearx2 = centroids.at<double>(cntmin2, 0);
						neary2 = centroids.at<double>(cntmin2, 1);
						cx2 = stats.at<int>(cntmin2, 0);
						cy2 = stats.at<int>(cntmin2, 1);
						cw2 = stats.at<int>(cntmin2, 2);
						ch2 = stats.at<int>(cntmin2, 3);
						//cout << abs(centerq.back() - c[0]) << endl;
						//cout << nearx << endl;
					}
				}
				//line(color, Point(centerq.back(), centeryq.back()), Point(c[0], c[1]), Scalar(255, 0, 255), 3);
				/*else {
					nearx = centerq.back();
					neary = centeryq.back();
					cx = cy = cw = ch = 0;
					cout << nearx << endl;
				}*/
				/*cout << sqrt((centerq.back() - c[0])*(centerq.back() - c[0]) + (centeryq.back() - c[1])*(centeryq.back() - c[1]))<<endl;*/
			}
			//cout << c[0] << endl;

		}
		//1
		centerq.push(nearx);
		centeryq.push(neary);
		if (abs(centerq.back() - centerq.front()) > 200) {
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
			cx = 0; cy = 0; cw = 0; ch = 0;
		}
		if (abs(centeryq.back() - centeryq.front()) > 60) {
			centeryq.push(centeryq.front());
			centeryq.pop();
			centeryq.push(centeryq.back());
			centeryq.pop();
		}
		centerqbacksave = centerq.back();
		centerysave = centeryq.back();

		circle(color, Point(centerqbacksave, centerysave), 2, Scalar(0, 0, 255), 2);
		rectangle(color, Rect(cx, cy, cw, ch), Scalar(0, 0, 255), 2);
		//2
		centerq2.push(nearx2);
		centeryq2.push(neary2);
		if (abs(centerq2.back() - centerq2.front()) > 200) {
			centerq2.push(centerq2.front());
			centerq2.pop();
			centerq2.push(centerq2.back());
			centerq2.pop();
			cx2 = 0; cy2 = 0; cw2 = 0; ch2 = 0;
		}
		if (abs(centeryq2.back() - centeryq2.front()) > 60) {
			centeryq2.push(centeryq2.front());
			centeryq2.pop();
			centeryq2.push(centeryq2.back());
			centeryq2.pop();
		}
		centerqbacksave2 = centerq2.back();
		centerysave2 = centeryq2.back();
		circle(color, Point(centerqbacksave2, centerysave2), 2, Scalar(0, 0, 255), 2);
		rectangle(color, Rect(cx2, cy2, cw2, ch2), Scalar(0, 0, 255), 2);

		line(color, Point(centerq.back(), centeryq.back()), Point(centerq2.back(), centeryq2.back()), Scalar(255, 0, 255), 2);
		line(color, Point(320, 0), Point(320, 90), Scalar(0, 255, 255), 2);
		if (centerysave2 >= centerysave)
			circle(color, Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave2 - centerysave) / 2 + centerysave), 2, Scalar(255, 255, 255), 2);
		else circle(color, Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave - centerysave2) / 2 + centerysave2), 2, Scalar(255, 255, 255), 2);

		//cout << centerq.back() << ", " << centerq.front() << endl;
		//cout << centermin << endl;
		err = cameracentroids.x - ((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave);
		lvel = 200 - gain * err;
		rvel = -(200 + gain * err);

		//line(color, Point(centerq.back(), centeryq.back()), Point(centerq2.back(), centeryq2.back()), Scalar(255, 0, 255), 3);
		//cout << sqrt((centerq.back() - centerq2.back()) * (centerq.back() - centerq2.back()) + (centeryq.back() - centeryq2.back()) * (centeryq.back() - centeryq2.back())) << endl;
		
		//1
		centerq.pop();
		centeryq.pop();

		//2
		centerq2.pop();
		centeryq2.pop();

		//centercnt++;
		//circle(color, cameracentroids, 2, Scalar(255, 0, 0), 2);
		writer1 << frame;
		writer2 << color;
		if (spress)
		{
			if (!mx.setVelocity(lvel, rvel)) { cout << "setVelocity error" << endl; return -1; }
		}
		usleep(20*1000);
		
		
		gettimeofday(&end1, NULL);
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
		cout << "err : " << err;
		cout << ", lvel : " << lvel;
		cout << ", rvel :" << rvel;
		cout << ", time : " << diff1 << endl;
		/*double time = (double)(end - start) / CLOCKS_PER_SEC;
		cout << time << endl;*/
	}
    mx.close();
	return 0;
}