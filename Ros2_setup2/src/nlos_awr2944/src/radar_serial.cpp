#include "nlos_awr2944/radar_driver_core.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cerrno>  // errno 사용을 위해 필수

// [중요] termios2 구조체 직접 정의 (헤더 충돌 방지)
struct termios2 {
    tcflag_t c_iflag;       /* input mode flags */
    tcflag_t c_oflag;       /* output mode flags */
    tcflag_t c_cflag;       /* control mode flags */
    tcflag_t c_lflag;       /* local mode flags */
    cc_t c_line;            /* line discipline */
    cc_t c_cc[19];          /* control characters */
    speed_t c_ispeed;       /* input speed */
    speed_t c_ospeed;       /* output speed */
};

#ifndef BOTHER
#define BOTHER 0010000
#endif

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif

#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif

namespace nlos_awr2944
{

namespace serial
{

// ===========================================
// 시리얼 포트 열기
// ===========================================
int openSerialPort(const std::string& port, int baudrate, double timeout_sec)
{
    // 디버그 로그 출력
    std::cout << "[Serial] Opening port: " << port << " at baud: " << baudrate << std::endl;

    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        // 실패 원인 출력
        std::cerr << "[Serial] Error opening port " << port << ": " 
                  << std::strerror(errno) << " (Error Code: " << errno << ")" << std::endl;
        return -1;
    }

    // [복구됨] termios 구조체 선언 및 초기화 (이 부분이 빠져있었습니다!)
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[Serial] Error getting attributes: " << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // 8N1 설정
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control

    // Raw 입력 모드
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw 출력 모드
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // 타임아웃 설정 (1/10초 단위)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>(timeout_sec * 10);

    // 설정 적용
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[Serial] Error setting attributes: " << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // 2. 비표준 보레이트(3125000 등) 지원을 위한 termios2 사용
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) == 0) {
        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;    // Custom Baudrate 플래그
        tio.c_ospeed = baudrate;  // 입력받은 숫자 그대로 사용 (예: 3125000)
        tio.c_ispeed = baudrate;

        if (ioctl(fd, TCSETS2, &tio) != 0) {
            std::cerr << "[Serial] Failed to set custom baudrate: " << baudrate 
                      << " Error: " << std::strerror(errno) << std::endl;
            close(fd);
            return -1;
        }
    } else {
        std::cerr << "[Serial] Error getting termios2 (TCGETS2): " << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // 비블로킹 해제 (read 시 타임아웃 적용을 위해)
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    // 버퍼 플러시
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// ===========================================
// 시리얼 데이터 쓰기
// ===========================================
ssize_t writeSerial(int fd, const void* data, size_t len)
{
    return write(fd, data, len);
}

// ===========================================
// 시리얼 데이터 읽기
// ===========================================
ssize_t readSerial(int fd, void* data, size_t max_len)
{
    return read(fd, data, max_len);
}

// ===========================================
// 사용 가능한 바이트 수 확인
// ===========================================
int availableBytes(int fd)
{
    int bytes_available = 0;
    ioctl(fd, FIONREAD, &bytes_available);
    return bytes_available;
}

// ===========================================
// 시리얼 포트 닫기
// ===========================================
void closeSerialPort(int fd)
{
    if (fd >= 0) {
        tcflush(fd, TCIOFLUSH);
        close(fd);
    }
}

// ===========================================
// 버퍼 플러시
// ===========================================
void flushSerial(int fd)
{
    tcflush(fd, TCIOFLUSH);
}

}  // namespace serial
}  // namespace nlos_awr2944