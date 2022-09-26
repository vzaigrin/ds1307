#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dev/iicbus/iic.h>
#include <time.h>

#define READ 1
#define WRITE 0


static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }


void usage ( char *pname )  {
  
  printf( "Usage: %s -r [-a <addr>] [-f <iic-dev>]\n", pname );
  printf( "       %s -s [-a <addr>] [-f <iic-dev>]\n", pname );
  exit(1);
}


char i2c_read ( int fd, uint16_t slave, uint8_t offset, uint8_t *buf )  {

  struct iic_msg msg[2];
  struct iic_rdwr_data rdwr;

  msg[0].slave = slave << 1 | !IIC_M_RD;
  msg[0].flags = !IIC_M_RD;
  msg[0].len = sizeof( uint8_t );
  msg[0].buf = &offset;

  msg[1].slave = slave << 1 | IIC_M_RD;
  msg[1].flags = IIC_M_RD;
  msg[1].len = sizeof( uint8_t );
  msg[1].buf = buf;

  rdwr.msgs = msg;
  rdwr.nmsgs = 2;

  if ( ioctl(fd, I2CRDWR, &rdwr) < 0 )  {
    perror("I2CRDWR");
    return(-1);
  }
  
  return(0);
}


char i2c_write ( int fd, uint16_t slave, uint8_t offset, uint8_t val )  {

  uint8_t buf[2];
  struct iic_msg msg;
  struct iic_rdwr_data rdwr;

  buf[0] = offset;
  buf[1] = val;
  msg.slave = slave << 1 | !IIC_M_RD;
  msg.flags = !IIC_M_RD;
  msg.len = 2*sizeof( uint8_t );
  msg.buf = buf;

  rdwr.msgs = &msg;
  rdwr.nmsgs = 1;

  if ( ioctl(fd, I2CRDWR, &rdwr) < 0 )  {
    perror("I2CRDWR");
    return(-1);
  }
  
  return(0);
}


int main ( int argc, char *argv[] )  {

  int i, ch, fd, fl;
  int ss, mm, hh, wd, d, m, y;
  char dev[] = "/dev/iic0";
  uint8_t buf;
  uint16_t slave = 0x68;
  time_t tloc;
  struct tm *ptm;


  if ( argc == 1 )  usage(argv[0]);

  while ( (ch = getopt(argc, argv, "hrsa:f:")) != -1 )  {
    switch (ch)  {
      case 'r': fl = READ;
                break;
      case 's': fl = WRITE;
                break;
      case 'a': sscanf( optarg, "%hX", &slave );
                break;
      case 'f': strcpy( dev, optarg );
                break;
      case 'h':
      default:  usage(argv[0]);
    }
  }
  
  if ( (fd = open(dev, O_RDWR)) < 0 )  {
    perror("open");
    exit(-1);
  }

  if ( fl == WRITE )  {

    tloc = time( &tloc );
    ptm = gmtime( &tloc );

    i2c_write( fd, slave, 0, bin2bcd(ptm->tm_sec) );
    i2c_write( fd, slave, 1, bin2bcd(ptm->tm_min) );
    i2c_write( fd, slave, 2, bin2bcd(ptm->tm_hour) );
    i2c_write( fd, slave, 3, bin2bcd(ptm->tm_wday+1) );
    i2c_write( fd, slave, 4, bin2bcd(ptm->tm_mday) );
    i2c_write( fd, slave, 5, bin2bcd(ptm->tm_mon+1) );
    i2c_write( fd, slave, 6, bin2bcd(ptm->tm_year-100) );

    exit(0);
  }

  if ( fl == READ )  {

    i2c_read( fd, slave, 0, &buf );
    ss = bcd2bin(buf & 0x7F);

    i2c_read( fd, slave, 1, &buf );
    mm = bcd2bin(buf & 0x7F);

    i2c_read( fd, slave, 2, &buf );
    hh = bcd2bin(buf & 0x3F);

    i2c_read( fd, slave, 3, &buf );
    wd = bcd2bin((buf & 0x07) - 1);

    i2c_read( fd, slave, 4, &buf );
    d = bcd2bin(buf & 0x3F);

    i2c_read( fd, slave, 5, &buf );
    m = bcd2bin(buf & 0x1F);

    i2c_read( fd, slave, 6, &buf );
    y = 2000 + bcd2bin(buf);

    printf("%.2d:%.2d:%.2d %.2d/%.2d/%.4d UTC\n", hh, mm, ss, d, m, y );
    
    exit(0);
  }

  exit(0);

}
