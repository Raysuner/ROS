#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>


std::string result;
int flag = 0;

void httpResponse(std::string& Text)
{
	Json::Value root;
	Json::Reader reader;
	bool success = reader.parse(Text,root);
	if(!success)
	{
		std::cout << "Failed" << std::endl;
	}
	Json::Value code = root["code"];
	Json::Value text = root["text"];
	result = text.asString();
	flag = 1;
	std::cout << "response code :" << code << std::endl;
	std::cout << "response text :" << text << std::endl;
}

int writer(char *p,size_t size,size_t nmemb, std::string* writerBuffer)
{
	if(writerBuffer == NULL)
	{
		return -1;
	}
	int len = size * nmemb;
	writerBuffer -> append(p,len);
	return len;
}

void httpRequest(std::string jsonText)
{
	std::string buffer;
	CURL* pcurl = NULL;
	CURLcode res;
	curl_global_init(CURL_GLOBAL_ALL);
	pcurl = curl_easy_init();
	if(pcurl)
	{
		curl_easy_setopt(pcurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");  //设置需要接受我们post的URL
		curl_easy_setopt(pcurl, CURLOPT_TIMEOUT, 5); //设置超时时间
		// 设置curl http header
		curl_slist *plist = curl_slist_append(NULL, "Content-Type:application/json; charset = UTF-8");
		curl_easy_setopt(pcurl, CURLOPT_HTTPHEADER, plist);
		// set curl post content
		curl_easy_setopt(pcurl, CURLOPT_POSTFIELDS, jsonText.c_str());
		curl_easy_setopt(pcurl, CURLOPT_WRITEFUNCTION,writer);
		curl_easy_setopt(pcurl, CURLOPT_WRITEDATA, &buffer);
		res = curl_easy_perform(pcurl);
	}
	if(res != CURLE_OK)
	{
		std::cout << "perform failed" << curl_easy_strerror(res) << std::endl;
	}
	curl_easy_cleanup(pcurl);
	curl_global_cleanup();
	if(buffer.empty())
	{
		std::cout << "!!!ERROR" << std::endl;
	}
	else
	{
		httpResponse(buffer);
	}
}

void msgToJson(std::string Text)
{
	std::string jsonText = "{";
	jsonText += "\"key\" : \"c967236c625e46938abaf1e45c3b7dc6\",";
	jsonText += "\"info\" : ";
	jsonText += "\"";
	jsonText += Text;
	jsonText += "\"";
	jsonText += "}";

	std::cout << "post jsonText : " << jsonText << std::endl;
	httpRequest(jsonText);
}

void tulingCallback(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "消息内容：" << msg -> data << std::endl;
	msgToJson(msg -> data);
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "tuling_node");
	ros::NodeHandle n;
	ros::Subscriber tuling_sub = n.subscribe("/tl_nlu",10,tulingCallback);
	ros::Publisher tuling_pub = n.advertise<std_msgs::String>("/xf_tts",10);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		if(flag)
		{
			std_msgs::String msg;
			msg.data = result;
			tuling_pub.publish(msg);
			flag = 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


