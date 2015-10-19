#include "tcp_ip_client.h"

#include "memory.h"
#include <arpa/inet.h>
#include "stdlib.h"
#include "unistd.h"

using namespace std;

namespace QuadLib{

std::string toString(QuadLib::TcpIpReturnValue stat){
	switch(stat){
		case QuadLib::RETURN_OK:
			return "OK";
			break;
		case QuadLib::RETURN_ERR:
			return "RETURN_ERR";
			break;
		case QuadLib::RETURN_NO_IP:
			return "RETURN_NO_IP";
			break;
		case QuadLib::RETURN_NO_PORT:
			return "RETURN_NO_PORT";
			break;
		case QuadLib::RETURN_SOCKET_ERR:
			return "RETURN_SOCKET_ERR";
			break;
		case QuadLib::RETURN_CONNECT_ERR:
			return "RETURN_CONNECT_ERR";
			break;
		case QuadLib::RETURN_READ_BUF_FULL:
			return "RETURN_READ_BUF_FULL";
			break;
	}
	return "unknow";
}


TcpIpClient::TcpIpClient(unsigned int n_buf,unsigned int n_buf_temp, const std::string & endChars){
	ip_address="";
	tcp_port="";
	buffer=new CharCircularBuffer(n_buf,endChars.c_str());
	fd=-1;
	tmp_buf_len=n_buf_temp;
	tmp_buf= new char [tmp_buf_len];
}

TcpIpClient::~TcpIpClient(){
	closeConnection();
	delete buffer;
	delete [] tmp_buf;
}
	
TcpIpReturnValue TcpIpClient::openConnection(){
	closeConnection();	
	return doConnection();
}
	
TcpIpReturnValue TcpIpClient::openConnection(std::string ip_address,std::string tcp_port){
	closeConnection();
	
	this->ip_address=ip_address;
	this->tcp_port=tcp_port;
	
	return doConnection();
}

TcpIpReturnValue TcpIpClient::doConnection(){
	int ret=0;
	
	if(ip_address.compare("")==0){
		return RETURN_NO_IP;
	}
	if(tcp_port.compare("")==0){
		return RETURN_NO_PORT;
	}
	
	bzero((char *) &server, sizeof(server));
    server.sin_family 		= AF_INET;
	server.sin_addr.s_addr 	= inet_addr(ip_address.c_str());
	server.sin_port 		= htons((unsigned short)atoi(tcp_port.c_str()));
	
	fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd<0)return RETURN_SOCKET_ERR;
	
	ret=connect(fd,(struct sockaddr *)  &server, sizeof(server));
	if (ret<0) return RETURN_CONNECT_ERR;

	set_fd(fd);
	
	buffer->reset();

	return RETURN_OK;	
}

TcpIpReturnValue TcpIpClient::reConnect(){
	closeConnection();
	return doConnection();
}

TcpIpReturnValue TcpIpClient::closeConnection(){
	if(fd>=0){
		close(fd);
		fd=-1;
		set_fd(-1);
		return RETURN_OK;
	}
	return RETURN_ERR;
}

TcpIpReturnValue TcpIpClient::sendMsg(std::string s){
	if(fd<0)return RETURN_SOCKET_ERR;
	if (send(fd, s.c_str(), s.size(), 0) < 0) {
		return RETURN_ERR;		
	}
	return RETURN_OK;	
}

TcpIpReturnValue TcpIpClient::readMsg(){
	if(fd<0)return RETURN_SOCKET_ERR;
	int count;
	count=recv(fd,tmp_buf,tmp_buf_len, 0);
	if(count<=0)
	  //printf("READ::tmp_buf[%d]\n",count);
	  return RETURN_ERR;		
	if(buffer->addNChar(tmp_buf,count)!=count){
		return RETURN_READ_BUF_FULL;
	}
	return RETURN_OK;	
}

TcpIpReturnValue TcpIpClient::readMsg(struct timeval &tout){

  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tout,sizeof(struct timeval));

  if(fd<0)return RETURN_SOCKET_ERR;
  int count;
  count=recv(fd,tmp_buf,tmp_buf_len, 0);
  if(count<=0)
    //printf("READ::tmp_buf[%d]\n",count);
    return RETURN_ERR;
  if(buffer->addNChar(tmp_buf,count)!=count){
    return RETURN_READ_BUF_FULL;
  }

  return RETURN_OK;
}


}
