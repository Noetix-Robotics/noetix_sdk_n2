
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
//#include "analysis_data.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/select.h>
#include <errno.h>
#include <termios.h>
#include <chrono>
#include <pthread.h>
#include "common.h"
#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 1024
#define key1  0x8
#define key2  0x2
//#define key3  
//#define key4  
#define key5  0x3
#define key6  0x4
#define key7  0x5
#define key8  0x7
#define key9  0x9
#define key10  0xa
#define key11 0x6
#define key12  0xb
#define keystop  0x88
#define POLYNOMIAL 0x1021
#define POLY_INIT  0x0000
#define POLY_END   0x0000
namespace legged{

// struct joydata{
//     double axes[2];//horz,vert;
//     int button[14];
// };



typedef struct{
   uint8_t up;
   uint8_t down;
}key_status;
typedef struct{
    uint8_t type;
    uint8_t number;
    uint16_t value;
}js_t;

class RemoteDriver{
    enum class Mode : uint8_t
   {
      LIE,
      STAND,
      RUN,
      WALK,//NOCYCLE
      MOTION,
      JUMP,
      SINGLEJUMP,
      DEFAULT
      
   };
public:
    RemoteDriver()=default;
    ~RemoteDriver();

    bool init(std::string port,int baudrate);
    void setParam();

    const joydata getremotedata();
protected:
    bool initSerial(std::string port);
    void _spin();
    void spin();
    void run();
    uint16_t crc16(uint8_t *ptr, unsigned char len);
    void analysisdata(uint8_t *data);
    int port_read(int fd,unsigned char *buf,int len);

    float mapRange(float x,float x1,float x2,float y1,float y2);
private:
    
    std::string port_;                      //串口端口
	int baudrate_;                          //波特率
    unsigned char read_buf[256];
    int  serial_; 
    boost::shared_ptr<boost::circular_buffer<char> > data_buffer_ptr_;   
    joydata jsdata_;
    
    std::string data_;
    boost::mutex m_mutex_;  
    int buffer_size_;
    key_status keystatus[12];



    uint8_t keybuf[14];
    uint8_t walksendflag,motionsendflag,jumpsendflag,runsendflag,switchsendflag,startsendflag;
    // RingBuffer环形存储区，用于存储从串口中读出的数据
    

    /**/
    
    
    //State machine variables for spinOnce
    int bytes_;
 
    bool configured_;


    uint8_t  message_in_[DATA_BUF_SIZE];
    uint8_t  gps_buf[DATA_BUF_SIZE];
    uint32_t gps_buf_index;
    std::map<uint32_t, std::string> gps_raw;

    //查询参数返回值相关
    boost::mutex m_response_mutex_;  

    float cmddata_x;
    float cmddata_y;
    int connect_time;
    pthread_mutex_t wr_mutex;

    //数据处理线程
    // boost::thread deseralize_thread_;
    // boost::thread serialread_thread_;
    std::thread deseralize_thread_;
    std::thread serialread_thread_;
};

}
