#ifndef __GSLAM_UTILS_H__
#define __GSLAM_UTILS_H__

#include <stdint.h>
#include <string>
#include <vector>
#include <fstream>

namespace GSLAM {
namespace utils {


////////////////////////////////////////////////////////////////////////////////
///  string utils
////////////////////////////////////////////////////////////////////////////////
typedef std::vector<std::string> StringArray;


/**
 * @brief split text by delims
 *
 * @param intext        - [in] input string
 * @param delims        - [in] delims, eg.: ' ', '-'
 *
 * @return StringArray data
 */
StringArray split_text(const std::string &intext, const std::string &delims);

/**
 * @brief join text to form a string
 *
 * @param sa            - [in] input StringArray
 * @param delims        - [in] delims, eg.: ' ', '-'
 *
 * @return string data
 */
std::string join_text(const StringArray& sa, const std::string& delims);


/**
 * @brief convert given string to upper case (use itself)
 *
 * @param s             - [in,out] string
 *
 * @return converted string
 */
std::string& str_toupper(std::string &s);


/**
 * @brief convert given string to lower case (use itself)
 *
 * @param s             - [in,out] string
 *
 * @return converted string
 */
std::string& str_tolower(std::string &s);



int str_to_int(const std::string &s);
std::string int_to_str(const int &i);


/**
 * @brief trim a const string, return a trimed string
 *
 * @param s             - [in] string
 *
 * @return trimed string
 */
std::string trim(const std::string &s);

/**
 * @brief trim a string, return itself
 * @param s             - [in,out] input string, and trimed string
 *
 * @return trimed string
 */
std::string& trim2(std::string& s);


/**
 * @brief read contents of file
 *
 * @param fname             - [in] file name
 *
 * @return file contents
 */
std::string read_textFile(const std::string& fname);



////////////////////////////////////////////////////////////////////////////////
/// path utils
////////////////////////////////////////////////////////////////////////////////

std::string path_getFileName(const std::string& fname);
std::string path_getPathName(const std::string& fname);
std::string path_getFileBase(const std::string& fname);
std::string path_getFileExt(const std::string& fname);
std::string path_join(const std::string &p1, const std::string &p2);
std::string path_join(const std::string &p1, const std::string &p2, const std::string &p3);
StringArray path_split(const std::string &fname);
StringArray path_splitext(const std::string &fname);

int path_mkdir(const std::string& path);
int path_rmdir(const std::string& path);
int path_rmfile(const std::string& path);
int path_lsdir(const std::string &dir_name, StringArray &dl, int sortFiles=0);
int path_rename(const std::string &pathOld, const std::string &pathNew);

std::string path_getUserSettingsPath(void);
std::string path_getAllUserSettingsPath(void);


////////////////////////////////////////////////////////////////////////////////
/// system functions
////////////////////////////////////////////////////////////////////////////////

std::string exec_cmd(const char* cmd);


////////////////////////////////////////////////////////////////////////////////
/// time utils
////////////////////////////////////////////////////////////////////////////////

// get milli-second
uint64_t tm_get_millis(void);

// get milli-second
uint64_t tm_get_ms(void);

// get micro-second
uint64_t tm_get_us(void);

// get timestamp since 1970 (unit is second)
double tm_getTimeStamp(void);

// get timestamp since 1970 (unit is second) from string with format of strptime
double tm_getTimeStamp(const char* dtStr, const char* fmt);

// get UNIX timestamp since 1970 (unit is second)
uint32_t tm_getTimeStampUnix(void);

// sleep for t milli-seconds
void tm_sleep(uint32_t t);

// sleep for t micro-seconds
void tm_sleep_us(uint64_t t);



////////////////////////////////////////////////////////////////////////////////
/// computer feature
////////////////////////////////////////////////////////////////////////////////

bool get_cpu_id_by_asm(std::string & cpu_id);
int get_macAddress(std::vector<std::string>& macs);
int get_diskNumber(std::vector<std::string>& features);
int get_computerFeatures(std::vector<std::string>& features);
int net_gethostname(std::string& hn);


}} // end of namespace GSLAM::utils

#endif // end of __GSLAM_UTILS_H__
