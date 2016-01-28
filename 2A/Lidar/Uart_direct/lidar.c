#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>

#define AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH 22
#define AUSBEE_LIDAR_PICCOLO_DATA_LENGTH 4

struct ausbee_lidar_data{
	uint16_t angle; // Angle of the point
	int speed; // Speed of the Lidar
	int distance_mm; // Measured distance
	uint8_t signal_strength; // Strength of the signal
	uint8_t error; // 1 if an error is encountered
	uint8_t strengthWarning; // 1 if the signal strength is too low
	uint8_t error_code; // Error code in case of error
};

struct ausbee_lidar_distance{
	volatile uint8_t lock; // when non null, indicate that an interuption is writting in the table
	volatile uint16_t table[360]; // store the distance in mm of the laser ray for each angle (0 to 360°)
};

static uint8_t frame[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
volatile static struct ausbee_lidar_data data[AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
static struct ausbee_lidar_distance distance = {0};


// fonction priv�e
void ausbee_lidar_parse_piccolo();

// rajoute une donn�e dans le buffer
void ausbee_lidar_push_char(unsigned char rec) {

	// variable to know if the frame is at the beginning
	static uint8_t not_started = 1;

	//Index of the buffer
	static uint8_t index_buffer = 0;

	//if the beginning of the frame has not been reached
	if (not_started == 1) {
		//check if the frame is at the beginning
		if (rec == 0xFA) {
			not_started = 0;
			//add the element to buffer and increment the buffer
			frame[0] = (unsigned char) rec;
			index_buffer++;
		}
	} else {
		//add the element
		frame[index_buffer] = (unsigned char) rec;
		index_buffer++;
	}
	//reset variables
	if (index_buffer == AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH) {
		// parse the frame
		ausbee_lidar_parse_piccolo();

		//TODO xSemaphoreGive(USART1ReceiveHandle);
		index_buffer = 0;
		not_started = 1;
	}

}

// rempli le tableau data d'apr�s le buffer frame
void ausbee_lidar_parse_piccolo() {

	distance.lock = 1;

	for (int i = 0; i < AUSBEE_LIDAR_PICCOLO_DATA_LENGTH; i++) {
		data[i].angle = (frame[1] - 0xA0) * 4 + i; // Get angle for each data structure in degree
		data[i].speed = ((frame[2] << 8) + frame[3]) >> 6; // Get actual rotation speed of the LIDAR
		data[i].distance_mm =
				frame[4 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
						+ ((frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
								& 0x3F) << 8); // Get the measured distance for each data structure
		data[i].signal_strength =
				frame[6 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
						+ (frame[7 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] << 8); // Get signal strength

		// Check if no error occurs during measure
		if (frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] & 0x80) {
			data[i].error = 1;
			data[i].error_code =
					frame[4 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
		} else {
			data[i].error = 0;
			data[i].error_code = 0x0;
		}

		// Fill the strength warning flag in the data structure
		if (frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] & 0x40)
			data[i].strengthWarning = 1;
		else
			data[i].strengthWarning = 0;

		int index = data[i].angle;
		if (index >= 0 && index < 360) {
			distance.table[index] = data[i].distance_mm;
		}
	}

	distance.lock = 0;
}

// get the distance covered by the laser rays in mm
// angle is in degree (0 to 360�)
int ausbee_lidar_get_distance(uint16_t angle)
{
	if (!distance.lock && angle < 360)
	{
		return distance.table[angle];
	}
	else
	{
		printf("echec lidar get distance\n\r");
		return -1;
	}
}


int main(){
	unsigned char tmp;
	int donnee;

	  struct termios tios;

	int fd=open("/dev/ttyACM0", O_RDWR);
	tcgetattr(fd, &tios);
	cfsetispeed(&tios, B115200);
	cfsetospeed(&tios, B115200);
	
	if(fd < 0){
		printf("%s\n", strerror(errno));
		return 0;
	}

	while(1){
		read(fd, &tmp, sizeof(tmp));
		ausbee_lidar_push_char(tmp);

		for(int i=0;i<360;i++)
		{
			donnee=ausbee_lidar_get_distance(i);
			printf("%d %d\n", i, donnee);
		}
	puts("Baudrate");
	  speed_t ispeed = cfgetispeed(&tios);
	  speed_t ospeed = cfgetospeed(&tios);
	  printf("baud rate in: 0%o\n", ispeed);
	  printf("baud rate out: 0%o\n", ospeed);

		
	}

	return 1;	
}
