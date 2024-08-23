#include <iostream>
#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>

int sockfd = -1;
struct sockaddr_in servaddr;

static __attribute__((constructor)) void init(void){
  /*
  // Create socket
  std::cout << "1" << std::endl;
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cerr << "Socket creation failed" << std::endl;
    exit(1);
  }
  
  std::cout << "2" << std::endl;
  memset(&servaddr, 0, sizeof(servaddr));

  // Server information
  std::cout << "3" << std::endl;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(65432);
  if (inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr) <= 0) {
    std::cerr << "Invalid address/ Address not supported" << std::endl;
    exit(1);
  }

  // Connect to server
  std::cout << "4" << std::endl;
  if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "Connection Failed" << std::endl;
    exit(1);
  }
  //*/
}

void injected_function(unsigned long i){
  static auto start = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  static long long prev_microseconds = microseconds;
  static long long prev_i = i;
  std::cout << "injected function " << microseconds << " " << (microseconds - prev_microseconds) << " " << i << " " << (i - prev_i) << " " << (i - microseconds) << std::endl;
  std::cout << "size: " << sizeof(unsigned long) << std::endl;
  prev_i = i;
  prev_microseconds = microseconds;


  if (sockfd == -1){
    // Create socket
    std::cout << "1" << std::endl;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      std::cerr << "Socket creation failed" << std::endl;
      exit(1);
    }
  
    std::cout << "2" << std::endl;
    memset(&servaddr, 0, sizeof(servaddr));

    // Server information
    std::cout << "3" << std::endl;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(65432);
    if (inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr) <= 0) {
      std::cerr << "Invalid address/ Address not supported" << std::endl;
      exit(1);
    }

    // Connect to server
    std::cout << "4" << std::endl;
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
      std::cerr << "Connection Failed" << std::endl;
      exit(1);
    }
  }
  
  // Send message
  //*
  char message_type = 't';
  uint8_t message[16];//sizeof(message_type) + sizeof(i)];
  std::memcpy(&message[0], &message_type, sizeof(message_type));
  std::memcpy(&message[sizeof(i)], &i, sizeof(i));
  send(sockfd, message, sizeof(message), 0);
  //send(sockfd, &i, sizeof(i), 0);

  // Receive response
  char buffer[1024] = {0};
  int time_to_sleep;
  //int valread = read(sockfd, buffer, 1024);
  int valread = read(sockfd, &time_to_sleep, sizeof(time_to_sleep));
  //std::cout << "Server: " << buffer << std::endl;
  std::cout << "Server: " << time_to_sleep << std::endl;
  if(time_to_sleep > 0)
    usleep(time_to_sleep);
  //*/
}


static __attribute__((destructor)) void end(void){
  close(sockfd);
}
