/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

int flag = 0 ;
bool Recflag = true;
std::string data = "";

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

static void show_result(char *str, char is_over)
{
	printf("Result: [ %s ]\n", str);
	if(is_over)
	{
		putchar('\n');
	}
	std::string s = str;
	data  = s;
	flag = 1;		
}

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = static_cast<char *>((realloc(g_result, g_buffersize + BUFFER_SIZE)));
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}

void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}

void on_speech_end(int reason)  //写入本次识别的音频发生错误或检测到VAD时才调用
{
	if (reason == END_REASON_VAD_DETECT)
	{
		printf("\nSpeaking done \n");
		Recflag = false;  // 侦听结束
	}
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* 持续侦听 */
	while(Recflag)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

void audioOrMic()
{
	int ret = MSP_SUCCESS;
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 5a47a36b, work_dir = .";
	//int aud_src = 0; /* from mic or file */

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit;
		return; // login fail, exit the program
	}

	demo_mic(session_begin_params);
exit:
	MSPLogout();
}
/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
void iatCallback(const std_msgs::String::ConstPtr& msg)
{
	Recflag = true;  // 每次调用回调函数就准备进行语音侦听
	if(msg -> data == "start") // 只有当输入唤醒词"start"时才进行语音识别
	{
		printf("<<<<<<< start >>>>>>>");
		audioOrMic();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"xf_asr_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/xf_asr",1,iatCallback);
	ros::Publisher pub = n.advertise<std_msgs::String>("/xf_tts",10);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		if(flag)
		{
			std_msgs::String msg;
			msg.data = data;
			pub.publish(msg);
			flag = 0;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
		
	MSPLogout(); // Logout...
	printf("LogOut>>>>>>>>>>>>\n");
	return 0;
}
