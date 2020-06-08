//
// Takes input from Vodafone AllthinksTalk and sent to Luftdaten
//
// version 1.0 / June 2020 / Paul van Haastrecht
// * initial version
//
//*****************************************************************************
//
// Copyright (c) 2020, Paul van Haastrecht
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// NO support, enjoy and good luck !!
//
//*****************************************************************************

# include <getopt.h>
# include <signal.h>
# include <stdarg.h>
# include <stdlib.h>
# include <stdio.h>
# include <string.h>
# include <time.h>
# include <unistd.h>
# include<sys/wait.h>  // wait
# include <sys/types.h>// open
# include <sys/stat.h> // open
# include <fcntl.h>    // open
# include <sys/time.h> // setItemer

#define SOFTWARE_VERSION "Version 1.0"

#define MAXFILE 200
char ConfigFile[MAXFILE] = {0};         // holds the configuration filename

// read from configuration file
char EncodingFile[MAXFILE] = {0};       // Encode/decode info between ATT and Luftdaten
char SaveFile[MAXFILE] = {0};           // name for local file storage (if wanted)
char FileExt[20] = {0};                 // extension on local file (if wanted)
char luftdatenUrl[MAXFILE] = {0};       // Luftdaten URL (if wanted)
char luftdatenSensorUID[MAXFILE] = {0}; // the Sensor-UID on Luftdaten
char luftdatenTimeout[5] = "10";        // timeout in seconds
int verbose = 0;                        // enable / disable debug information
int TimeOut = 10;                       // Max timeout between first character and sending

// encoding variables
struct enc {
    char    ATTsearch[20];              // Identify received ATT value
    int     LuftDatenPin;               // Forward to Luftdaten sensor-pin
    char    LuftDaten_value_type[20];   // Forward to Luftdaten as value-type for that pin
    bool    AlreadyInData;              // check as Luftdaten fails if same value-type is 2 times in data.
};

#define MAXENC 20                       // maximum encoding /decoding entries
struct enc encoding[20];                // place to store
int enc_offset=0;                       // number encoding / decoding entries in use

// others
int CurrentPin = 0;                     // remember current Luftdaten sensor-pin
char input[500];                        // store input
char data_output[1000];                 // store data output for curl
bool NewDataString = true;              // Indicate to create data header for CURL output
char progname[20];                      // used in usage ()
FILE *fileID = NULL;                    // open file pointer
char StorageFile[MAXFILE] = {0};        // name for local file storage (base_name + extension)
volatile int count = 0;                 // counter for input

/*! to display in color  */

// color display enable
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define WHITE   5

#define REDSTR "\e[1;31m%s\e[00m"
#define GRNSTR "\e[1;92m%s\e[00m"
#define YLWSTR "\e[1;93m%s\e[00m"
#define BLUSTR "\e[1;34m%s\e[00m"

// disable color output
bool NoColor = false;

/*********************************************************************
 * @brief Display in color
 * @param format : Message to display and optional arguments
 *                 same as printf
 * @param level :  1 = RED, 2 = GREEN, 3 = YELLOW 4 = BLUE 5 = WHITE
 *
 * if NoColor was set, output is always WHITE.
 *********************************************************************/
void p_printf(int level, const char *format, ...) {

    char    *col;
    int     coll=level;
    va_list arg;

    //allocate memory
    col = (char *) malloc(strlen(format) + 20);

    if (NoColor) coll = WHITE;

    switch(coll)  {
        case RED:
            sprintf(col,REDSTR, format);
            break;
        case GREEN:
            sprintf(col,GRNSTR, format);
            break;
        case YELLOW:
            sprintf(col,YLWSTR, format);
            break;
        case BLUE:
            sprintf(col,BLUSTR, format);
            break;
        default:
            sprintf(col,"%s",format);
    }

    va_start (arg, format);
    vfprintf (stdout, col, arg);
    va_end (arg);

    fflush(stdout);

    // release memory
    free(col);
}
/************************************************
 * @brief Close local file with received data
 ************************************************/
void Savefile_close()
{
    if (fileID != NULL) {
        fflush(fileID);
        fclose(fileID);
        fileID = NULL;
    }
}

/***********************************************
*  @brief close hardware and program correctly
************************************************/
void closeout()
{
   p_printf(GREEN, "exiting ATT to Luftdaten\n");
   Savefile_close();
   exit(EXIT_SUCCESS);
}

/************************************************
 * @brief Read and parse the encoding/decoding file
 ************************************************/
void read_encoding_file()
{
    FILE *fp;
    char str[200], *p, t[5];
    int lineCount = 0, i;

    // check for encoding file
    if (strlen(EncodingFile) == 0){
        p_printf(RED, (char *) "No encoding file provided\n");
        exit(EXIT_FAILURE);
    }

    // open file
    fp = fopen(EncodingFile, "r");
    if (fp == NULL) {
        p_printf(RED, "Can not open encoding file %s.\n", EncodingFile);
        exit(EXIT_FAILURE);
    }

    // read each encoding line
    while(( fgets (str, 200, fp)) != NULL ) {

        // skip empty of comment line
        if ( strlen(str) < 3 || str[0] == '#') continue;

        if ((p = strstr(str,":")) == NULL) {
            p_printf(RED,"Error on line %d : %s. Skipping !\n", lineCount,str);
            continue;
        }

        *p++ = 0x0; // terminate selection argument

        // get the ATT selection argument
        strncpy(encoding[enc_offset].ATTsearch, str, 20);

        // get the Sensor_pin
        i = 0;
        while (*p != ':') t[i++] = *p++;
        t[i]=0x0;
        encoding[enc_offset].LuftDatenPin = (int)strtod(t, NULL);

        p++;  // skip :

        // get value_type
        i=0;
        while (*p != 0x0a && *p != 0x0d) {
            encoding[enc_offset].LuftDaten_value_type[i++] = *p++;
            if (i > 20) {
                p_printf(RED,"Exceeding maximum characters for Luftdaten value_type in encoding line %d\n", enc_offset);
                fclose(fp);
                exit(EXIT_FAILURE);
            }
        }

        encoding[enc_offset].AlreadyInData = false;

        if(verbose){
            p_printf(YELLOW, "Using encoding : ATT type %s, pin %d, value-type %s\n", encoding[enc_offset].ATTsearch,
            encoding[enc_offset].LuftDatenPin, encoding[enc_offset].LuftDaten_value_type);
        }

        if (++enc_offset > MAXENC) {
            p_printf(RED,"Exceeding maximum encoding entries %d\n", enc_offset);
            fclose(fp);
            exit(EXIT_FAILURE);
        }
    }

    if (verbose) p_printf(GREEN,"Encoding file %s has been read.\n\n",EncodingFile);

    fclose(fp);
}

/************************************************
 * @brief Read and parse the configuration file
 ************************************************/
void read_config_file()
{
    FILE *fp;
    char str[200], *p;
    uint i;

    // check for config file
    if (strlen(ConfigFile) == 0){
        p_printf(RED, "No configuration file provided\n");
        exit(EXIT_FAILURE);
    }

    // open file for reading
    fp = fopen(ConfigFile, "r");
    if (fp == NULL) {
        p_printf(RED, "Can not open configuration file %s\n", ConfigFile);
        exit(EXIT_FAILURE);
    }

    // read each line
    while(( fgets(str, 200, fp) ) != NULL ) {

        // skip empty of comment line
        if ( strlen(str) < 5 || str[0] == '#') continue;

        // remove NL or CR
        for (i = 0; i < strlen(str); i++) {
            if (str[i] == 0x0a || str[i] == 0x0d) {
                str[i] = 0x0;
                break;
            }
        }

        if ((p = strstr(str,"luftdaten.url=")) != NULL) {
            p += strlen("luftdaten.url=");
            strncpy(luftdatenUrl, p, MAXFILE);
        }
        else if ((p = strstr(str,"luftdaten.timeout=")) != NULL) {
            p += strlen("luftdaten.timeout=");
            strncpy(luftdatenTimeout, p, 3);
        }
        else if ((p = strstr(str,"storage.file=")) != NULL) {
            p += strlen("storage.file=");
            strncpy(SaveFile, p, MAXFILE);
        }
        else if ((p = strstr(str,"encoding=")) != NULL) {
            p += strlen("encoding=");
            strncpy(EncodingFile, p, MAXFILE);
        }
        else if ((p = strstr(str,"Sensor_UID=")) != NULL) {
            p += strlen("Sensor_UID=");
            strncpy(luftdatenSensorUID, p, MAXFILE);
        }
        else if ((p = strstr(str,"enable.dbg=")) != NULL) {
            p += strlen("enable.dbg=");
            verbose = (int) strtod(p, NULL);
        }
        else if ((p = strstr(str,"time_out=")) != NULL) {
            p += strlen("time_out=");
            TimeOut = (int) strtod(p, NULL);
        }
        else if ((p = strstr(str,"file.ext=")) != NULL) {
            p += strlen("file.ext=");
            strncpy(FileExt, p, 20);
        }
    }

    if (verbose) {
        p_printf(YELLOW,"Luftdaten URL      : %s\n",luftdatenUrl);
        p_printf(YELLOW,"Luftdaten timeout  : %s seconds\n",luftdatenTimeout);
        p_printf(YELLOW,"Luftdaten Sensor   : %s\n",luftdatenSensorUID);
        p_printf(YELLOW,"Encoding file      : %s\n",EncodingFile);

        if (strlen(SaveFile) > 0){
            p_printf(YELLOW,"Local storage file : %s\n",SaveFile);
            p_printf(YELLOW,"Storage file ext.  : %s\n",FileExt);
        }
        else
            p_printf(YELLOW,"Local storage file : not requested\n");

        p_printf(YELLOW,"Send Time_out      : %d seconds\n",TimeOut);
        p_printf(YELLOW,"debug message level: %d\n",verbose);
        p_printf(GREEN,"Configuration file %s has been read.\n\n",ConfigFile);
    }

    // close config file
    fclose(fp);
}

/***********************************************
 * @brief : create a configuration sample file
 ***********************************************/
void create_conf_file()
{
    FILE *fp;
    char inp[5];

    // open file to check it exists
    fp = fopen(ConfigFile, "r");
    if (fp != NULL) {
        fclose(fp);

        do {
            p_printf(RED, "configuration file %s exists. Overwrite:  yes or no ?\n", ConfigFile);
            fgets(inp,5,stdin);
            if (strstr (inp, "no") != NULL) exit(EXIT_FAILURE);
        } while (strstr (inp, "yes") == NULL);

    }

    // open for create or overwrite
    fp = fopen(ConfigFile, "w");
    if (fp == NULL) {
        p_printf(RED, "Can not create configuration file %s\n", ConfigFile);
        exit(EXIT_FAILURE);
    }

    fprintf(fp, "# configuration file for AllthingsTalk to Luftdaten\n");
    fprintf(fp, "# created with %s\n\n", SOFTWARE_VERSION);
    fprintf(fp, "# The payload encoding / decoding file\n");
    fprintf(fp, "encoding=att_to_luft\n\n");
    fprintf(fp, "# Luftdaten server URL (leave empty to disable sending)\n");
    fprintf(fp, "luftdaten.url=https://api.sensor.community/v1/push-sensor-data/\n\n");
    fprintf(fp, "# Luftdaten Sensor_UID (e.g. TTN-3646575)\n");
    fprintf(fp, "Sensor_UID=TTN-3646575\n\n");
    fprintf(fp, "#Luftdaten timeout (sec)\n");
    fprintf(fp, "luftdaten.timeout=10\n\n");
    fprintf(fp, "# Base name for local data files (leave empty to disable local filesaving)\n");
    fprintf(fp, "storage.file=/tmp/attol\n\n");
    fprintf(fp, "# Format date extension for local data file (leave empty to disable adding extension)\n");
    fprintf(fp, "# YY - year, MM - month, DD = day of month, HH - hour of day (0 - 23)\n");
    fprintf(fp, "# A new file will be created based on the lowest time element selected (e.g. YYMMDD - every day, YYMMDDHH - every hour)\n");
    fprintf(fp, "file.ext=YYMMDD\n\n");
    fprintf(fp, "# maximum timeout between receiving first data from ATT and sending to Luftdaten (seconds)\n");
    fprintf(fp, "time_out=10\n\n");
    fprintf(fp, "# Debug messages\n");
    fprintf(fp, "# 0 will show program log only, 1 will enable data flow messages, 2 will show more including debug curl\n");
    fprintf(fp, "enable.dbg=0\n");
    fclose(fp);

    p_printf(GREEN,"Configuration file %s has been created.\n",ConfigFile);
}

/*****************************************
 * @brief add file extension to local file
 *
 * format is read from the configuration file
 *****************************************/
void CreateFileExtension()
{
   char result[20],format[10] = {0},*p;
   time_t t;

   // if now timing extention request
   if (strlen(FileExt) == 0) {
        sprintf(StorageFile,"%s", SaveFile);
        return;
   }

   // Retrieve the current time
   t = time(NULL);

   p = &FileExt[0];

   while(*p != 0x0){
       if (*p == 'y' || *p == 'Y') sprintf(format, "%s%%y",format);
       else if (*p == 'm' || *p == 'M') sprintf(format, "%s%%m",format);
       else if (*p == 'd' || *p == 'D') sprintf(format, "%s%%d",format);
       else if (*p == 'h' || *p == 'H') sprintf(format, "%s%%H",format);

       p+=2;

       if (strlen(format) > sizeof(format)) {
          p_printf(RED,"File extension request is too long\n");
          exit(EXIT_FAILURE);
       }
   }

   /* Output format into the result string */
   strftime(result, sizeof(result), format, localtime(&t));

   sprintf(StorageFile,"%s.%s", SaveFile,result);
}

/************************************************
 * @brief Open local file to store received data
 ************************************************/
int Savefile_open()
{
    // create full file name with extension
    CreateFileExtension();

    if (fileID != NULL) Savefile_close();

    fileID = fopen(StorageFile,"a");         // append

    if (fileID == NULL) {
        if (verbose) p_printf(RED, "can not open Savefile %s\n", StorageFile);
        return(-1);
    }
    return(0);
}

/************************************************
 * @brief Display usage information
 ************************************************/
void usage()
{
    p_printf(YELLOW,"%s [options]  (%s) \n\n"

    "c           configuration file to read\n"
    "C           configuration file to create\n"
    "n           No color in output\n"
    "h or H      help text\n",
    progname, SOFTWARE_VERSION);
}


/************************************************
 * @brief Parse the command line options
 *
 * @param opt : the option code
 * @param option : pointer to argument provided with option code
 ************************************************/
void parse_cmdline(int opt, char *option)
{
    switch (opt) {

    case 'C' :  // create configuration file to create
        strncpy(ConfigFile, option, MAXFILE);
        create_conf_file();
        exit(EXIT_SUCCESS);
        break;
    case 'c':   // configuration file to read
        strncpy(ConfigFile, option, MAXFILE);
        break;
    case 'n':   // No color in output
        NoColor = true;
        break;
    default:
        usage();
        exit(EXIT_SUCCESS);
        break;
    }
}

#ifndef WIN32
/*********************************************************************
* @brief catch signals to close out correctly
* @param  sig_num : signal that was raised
*
**********************************************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        case SIGINT:
        case SIGKILL:
        case SIGABRT:
        case SIGTERM:
        default:
            closeout();
            break;
    }
}

/*****************************************
 * @brief setup signals
 *****************************************/
void set_signals()
{
    struct sigaction act;

    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);

    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

#endif // WIN32

/************************************************
 * @brief Send data to Luftdaten
 *
 * the curl string will look something like :
 * curl -X POST --header 'Content-Type: application/json' --header 'X-Pin: 1' --header 'X-Sensor: TTN-12345678'  --data '{
  "software_version": "version 1.0", "sensordatavalues":[{"value_type":"P1","value":"56.04"},{"value_type":"P2","value":"47.04"}] }'   https://api.sensor.community/v1/push-sensor-data/ --connect-timeout 10 -v
 ************************************************/
void SendToLuftdaten()
{
    pid_t child_pid, tpid;
    int child_status, i, fd = -1;
    char *argv[15];
    char programName[] = "curl";
    char header[] = "--header";
    char data[] = "--data";
    char xpin[20];
    char xsensor[50];

    // no sending if URL is empty
    if (strlen(luftdatenUrl) == 0) goto reset_only;

    // add trailer to data. Prepared for sending
    sprintf(data_output,"%s]}",data_output);

    if (verbose) p_printf(GREEN, "Sending to SensorID %s, pin %d, data: %s\n", luftdatenSensorUID, CurrentPin, data_output);

    // create curl string
    sprintf(xpin, "X-Pin: %d", CurrentPin);
    sprintf(xsensor, "X-Sensor: %s", luftdatenSensorUID);

    argv[0] = programName;
    argv[1] = header;
    argv[2] = (char *) "Content-Type: application/json";
    argv[3] = header;
    argv[4] = xpin;
    argv[5] = header;
    argv[6] = xsensor;
    argv[7] = data;
    argv[8] = data_output;
    argv[9] = luftdatenUrl;
    argv[10] = (char *) "--connect-timeout";
    argv[11] = luftdatenTimeout;

    if (verbose == 2) {
        argv[12] = (char *) "-v";
        argv[13] = NULL;
    }
    else {

        fd = open("/dev/null", O_WRONLY);
        if (fd == -1) {
            p_printf(RED, "Can not redirect output\n");
        }

        argv[12] = NULL;
    }

    // create child
    child_pid = fork();
    if(child_pid == 0) {
        /* This is done by the child process. */

        // redirect output
        if (fd != -1){
            if (dup2(fd, 1) < 0) perror("dup2");
            if (dup2(fd, 2) < 0) perror("dup2");
            close(fd);
        }

        // execute program
        execvp(programName, argv);

        // If execvp returns, it must have failed. This will not be shown
        // if redirection was done, but will in level 2 debug messages
        if (verbose) p_printf(RED, "Failed to run curl\n");
        exit(-1);
    }

    /* This is run by the parent.  */

    // if child's output is redirected
    if (fd != -1) close(fd);

    // Wait for the child to terminate.
    do {
        tpid = wait(&child_status);
    } while(tpid != child_pid);

    if (verbose) p_printf(YELLOW, "\ncurl returned %d\n",child_status);

reset_only:
    // empty data string
    data_output[0] = 0x0;

    // indicate new data string starting
    NewDataString = true;

    // reset sensor Pin
    CurrentPin = 0;

    // reset tracking to prevent double data for same value-type
    for (i = 0; i < enc_offset; i++) encoding[i].AlreadyInData = false;
}

/******************************************************************
 * @brief parse the received data and add to the data string to sent
 * to Luftdaten
 *
 * Input example :
 * device/HYTYs9Y757XSoNgRdsQRMTYYs0N/asset/M1/feed {"at":"2020-05-24T18:29:22.56146Z","value":"4.32"}
 *******************************************************************/
void parse_data_values(char *input)
{
    char *p, hay[10], value[30];
    int i,j;

    // look for asset string from ATT
    if ((p = strstr(input,"/asset/")) == NULL) return;

    // skip trailer
    p += strlen("/asset/");

    // extract ATT indicator
    i = 0;
    while(*p != '/') {
        hay[i++] = *p++;
        if (i > 10) return;     // prevent overflow
    }
    hay[i] = 0x0;

    // check for ATT indicator
    for (i = 0; i < enc_offset; i++) {

        // if match
        if (strcmp(hay, encoding[i].ATTsearch) == 0) {
            if ((p = strstr(input,"\"value\":")) == NULL) return;

            // extract value
            j = 0;
            while(*p != '}') {
                value[j++] = *p++;
                if (j > 30) return; // prevent overflow
            }
            value[j] = 0x0;

            // if sensor pin was set
            if (CurrentPin != 0){

                // if sensor pin changed from previous, we need to sent
                // current information
                if (encoding[i].LuftDatenPin != CurrentPin){
                    if (verbose) p_printf(GREEN,"New sensor-type / pin\n");
                    SendToLuftdaten();
                }

                // upload will fail to Luftdaten if same value_type is already
                // in the data string. In that case sent the current value first
                if (encoding[i].AlreadyInData){
                    if (verbose) p_printf(GREEN,"Same value-type already in data\n");
                    SendToLuftdaten();
                }
            }

            // starting new data string
            if (NewDataString) {

               /**'{"software_version": "Version 1.0", "sensordatavalues":[{"value_type":"P1","value":56.04},{"value_type":"P2","value":47.04}] }'*/
               // add data header
               sprintf(data_output,"{\"software_version\": \"%s\", \"sensordatavalues\":[", SOFTWARE_VERSION);

               NewDataString = false;
               CurrentPin = encoding[i].LuftDatenPin;
            }
            else {
                // appening extra comma before new item
                sprintf(data_output,"%s,", data_output);
            }

            // add data item
            sprintf(data_output,"%s{\"value_type\":\"%s\",%s}", data_output, encoding[i].LuftDaten_value_type,value);

            // indicate this value-type has been added to data.
            encoding[i].AlreadyInData = true;

            if (verbose) p_printf(BLUE,"Input added: %s\n", data_output);

            return;
        }
    }

    // when we get here NO match was detected
    if (verbose)  p_printf(YELLOW,"Warning could not handle the ATT value-type '%s' from the sentence\n\t%s\n",hay, input);
}

////////////////////////////////////////////////////////////////////
// timer functions
////////////////////////////////////////////////////////////////////
void (*timer_func_handler_pntr)(void);

/************************************************
 * @brief called with time expires
 ************************************************/
void timer_sig_handler(int arg)
{
  timer_func_handler_pntr();
}

/************************************************
 * @brief set one-off timer in seconds
 ************************************************/
int start_timer(int Sec, void (*timer_func_handler)(void))
{
  struct itimerval timervalue;
  struct sigaction new_handler, old_handler;

  timer_func_handler_pntr = timer_func_handler;

  timervalue.it_interval.tv_sec = 0;  // one off timer (no renew)
  timervalue.it_interval.tv_usec = 0;
  timervalue.it_value.tv_sec = Sec;
  timervalue.it_value.tv_usec = 0;
  if(setitimer(ITIMER_REAL, &timervalue, NULL))
  {
    printf("\nsetitimer() error\n");
    return(EXIT_FAILURE);
  }

  new_handler.sa_handler = &timer_sig_handler;
  new_handler.sa_flags = SA_NOMASK;
  if(sigaction(SIGALRM, &new_handler, &old_handler))
  {
    printf("\nsigaction() error\n");
    return(EXIT_FAILURE);
  }

  return(0);
}

/************************************************
 * @brief Handles check for sending data
 *
 * called from timer_sig_handler() as sent time-out expires
 ************************************************/
void check_sending(void)
{
    // if any data pending to be sent
    if (strlen(data_output) > 0)  {
        if (verbose) p_printf(GREEN, "Triggerd by Time-out\n");
        SendToLuftdaten();
    }
}

/**
 * @brief Main loop that will continue until program terminates
 */
void main_loop()
{
    p_printf(GREEN,"Starting mainloop.\n");
    if (verbose) p_printf(GREEN,"Software %s\n",SOFTWARE_VERSION);

    while(1) {

        // get character blocking
        int ch = getchar();

        // if not EOF (which happens sometimes..)
        if ( ch != -1 ) {

            // if first character to the input buffer
            // will be reset start each new line
            if (count == 0) {

                if (verbose > 1) p_printf(GREEN, "starting time-out\n");

                // set / update timer for TimeOut seconds (default 10 seconds) from now
                if (start_timer(TimeOut, &check_sending))
                {
                    p_printf(RED,"\nsetting time-out error\n");
                    exit(EXIT_FAILURE);
                }
            }

            // add to input
            input[count++] = ch;

            // end of line received
            if (ch == '\n') {

                // terminate line
                input[count] = 0x0;

                // save to file (if requested)
                if (strlen(SaveFile) > 0) {

                    if ( Savefile_open() == 0){
                        fputs(input, fileID);
                        Savefile_close();
                    }
                }

                // extended debug
                if (verbose > 1) p_printf(WHITE,"Received: %s", input);

                // parse for Luftdaten
                parse_data_values(input);

                // reset input buffer
                count = 0;
                input[count] = 0x0;
            }
        }
    }
}

/**
 * @brief Main : start of program
 */
int main(int argc, char *argv[])
{
    int opt;

#ifndef WIN32
    set_signals();
#endif // WIN32

    /* save name for (potential) usage display */
    strncpy(progname,argv[0],20);

    /* parse commandline */
    while ((opt = getopt(argc, argv, "nC:c:hH")) != -1) {
        parse_cmdline(opt, optarg);
    }

    // read config file
    read_config_file();

    // read encoding file
    read_encoding_file();

    // main loop
    main_loop();

    exit(EXIT_SUCCESS);
}
