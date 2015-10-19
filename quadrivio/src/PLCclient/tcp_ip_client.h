#ifndef TCP_IP_CLIENT_H_
#define TCP_IP_CLIENT_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string>
#include "CharCircularBuffer.h"
#include "SerialCommunication.h"


namespace QuadLib{

enum TcpIpReturnValue{
	RETURN_OK 			 = 0,
	RETURN_ERR 			 = -1,
	RETURN_NO_IP 		 = -2,
	RETURN_NO_PORT 		 = -3,
	RETURN_SOCKET_ERR 	 = -4,
	RETURN_CONNECT_ERR 	 = -5,
	RETURN_READ_BUF_FULL = -6
};

std::string toString(QuadLib::TcpIpReturnValue stat);

class TcpIpClient : public SerialCommunication{
	std::string ip_address;
	std::string tcp_port;
	int fd;
	struct sockaddr_in server;

	CharCircularBuffer * buffer;
	char * tmp_buf;
	unsigned int tmp_buf_len;

	TcpIpReturnValue doConnection();

public:
	TcpIpClient(unsigned int n_buf,unsigned int n_buf_temp, const std::string & endChars);
	~TcpIpClient();
	
	inline void setIp(std::string ip){ip_address=ip;};
	inline void setTcpPort(std::string port){tcp_port=port;};
	
	TcpIpReturnValue openConnection(std::string ip_address,std::string tcp_port);
	TcpIpReturnValue openConnection();
	TcpIpReturnValue reConnect();
	TcpIpReturnValue closeConnection();
	
	inline TcpIpReturnValue getStatus(){
		if(fd<0)
			return RETURN_ERR;
		return RETURN_OK;
	};

	TcpIpReturnValue sendMsg(std::string s);
	TcpIpReturnValue readMsg();
	TcpIpReturnValue readMsg(struct timeval &tout);
	
	inline CharCircularBuffer * getBuffer(){return buffer;};
	inline int getFd() {return fd;}; // return the connection file descriptor, for orogen
};

}

#endif
