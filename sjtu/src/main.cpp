﻿#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::dynamic;

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";

int main(int argc, char *argv[])
{
	std::cout << "new" << std::endl;

	xmlpath = xmlpath / xmlfile;
	//uixmlpath = uixmlpath / uixmlfile;
	std::cout << xmlpath << std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);


	//-------for sanxiang robot begin//
	cs.resetController(kaanh::createControllerSanXiang().release());
	cs.resetModel(kaanh::createModelSanXiang().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for sanxiang robot end//


	cs.loadXmlFile(xmlpath.string().c_str());
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	cs.start();

	//Start Web Socket//
	cs.startWebSock("5866");

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
