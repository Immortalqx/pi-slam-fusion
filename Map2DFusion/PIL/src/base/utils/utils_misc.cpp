/******************************************************************************

  Robot Toolkit ++ (RTK++)

  Copyright (c) 2007-2013 Shuhui Bu <bushuhui@nwpu.edu.cn>
  http://www.adv-ci.com

  ----------------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

*******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#include <math.h>
#include <complex.h>
#include <float.h>

#ifdef PIL_LINUX
#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#endif

#ifdef PIL_WINDOWS
#include <windows.h>
#endif

#include <errno.h>

#include <assert.h>
#include <inttypes.h>

#ifdef __SSSE3__
#include <tmmintrin.h>
#endif

#include <string>
#include <vector>
#include <algorithm>

#include "base/types/types.h"
#include "base/time/DateTime.h"
#include "base/time/Time.h"
#include "base/system/file_path/file_path.h"

#include "utils_str.h"
#include "utils_misc.h"



using namespace std;

namespace pi {


////////////////////////////////////////////////////////////////////////////////
/// arguments functions
////////////////////////////////////////////////////////////////////////////////

void save_arguments(int argc, char *argv[], string &fname)
{
    string      fn;
    FILE        *fp;
    int         i;
    tm          *now;
    time_t      t;
    char        str_time[200];


    fn = fname + "_args.txt";
    fp = fopen(fn.c_str(), "a+"); ASSERT(fp);

    // get current time
    time(&t);
    now = localtime(&t);
    strftime(str_time, 200, "%Y-%m-%d %H:%M:%S", now);

    fprintf(fp, "--------------- %s ---------------\n", str_time);

    for(i=0; i<argc; i++)
        fprintf(fp, "%s ", argv[i]);

    fprintf(fp, "\n\n");

    fclose(fp);
}


////////////////////////////////////////////////////////////////////////////////
/// file & path functions
////////////////////////////////////////////////////////////////////////////////

long filelength(FILE *fp)
{
    long    len;

    if( fp == NULL )
        return 0;

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    return len;
}

long filelength(const char *fname)
{
    FILE    *fp;
    long    len;

    fp = fopen(fname, "r");
    if( fp == NULL )
        return 0;

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fclose(fp);

    return len;
}


const std::string auto_filename(const char *fn_base)
{
    DateTime    t;

    t.setCurrentDateTime();

    if( fn_base == NULL ) {
        return fmt::sprintf("autosave_%04d%02d%02d_%02d%02d%02d",
                            t.year, t.month, t.day,
                            t.hour, t.min, t.sec);
    } else{
        return fmt::sprintf("%s_%04d%02d%02d_%02d%02d%02d",
                            fn_base,
                            t.year, t.month, t.day,
                            t.hour, t.min, t.sec);
    }
}

const std::string auto_filename(const std::string &fn_base)
{
    DateTime    t;

    t.setCurrentDateTime();

    if( fn_base.size() == 0 ) {
        return fmt::sprintf("autosave_%04d%02d%02d_%02d%02d%02d",
                            t.year, t.month, t.day,
                            t.hour, t.min, t.sec);
    } else{
        return fmt::sprintf("%s_%04d%02d%02d_%02d%02d%02d",
                            fn_base,
                            t.year, t.month, t.day,
                            t.hour, t.min, t.sec);
    }
}





////////////////////////////////////////////////////////////////////////////////
/// text file functions
////////////////////////////////////////////////////////////////////////////////

int readlines(const char *fn, StringArray &lns, int buf_len)
{
    FILE    *fp=NULL;

    char    *buf;
    string  s;

    // clear old data
    lns.clear();

    // alloc buffer
    buf = new char[buf_len];

    // open file
    fp = fopen(fn, "r");
    if( fp == NULL ) {
        ASSERT(fp);
        return -1;
    }

    while( !feof(fp) ) {
        // read a line
        if( NULL == fgets(buf, buf_len, fp) )
            break;

        // remove blank & CR
        s = trim(buf);

        // skip blank line
        if( s.size() < 1 )
            continue;

        // add to list
        lns.push_back(s);
    }

    // close file
    fclose(fp);

    // free array
    delete [] buf;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// array write/read functions
////////////////////////////////////////////////////////////////////////////////

int save_darray(const char *fn, ru64 n, double *d)
{
    FILE        *fp;

    fp = fopen(fn, "w"); ASSERT(fp);

    fwrite(&n, sizeof(ru64),    1, fp);
    fwrite(d,  sizeof(double)*n,    1, fp);
    fclose(fp);

    return 0;
}

int load_darray(const char *fn, ru64 &n, double **d)
{
    FILE        *fp;
    double      *d_;
    int         i;

    fp = fopen(fn, "r"); ASSERT(fp);

    i = fread(&n, sizeof(ru64), 1, fp);

    d_ = new double[n];
    i = fread(d_, sizeof(double)*n, 1, fp);
    fclose(fp);

    *d = d_;

    return 0;
}

int save_farray(const char *fn, ru64 n, float *d)
{
    FILE        *fp;

    fp = fopen(fn, "w"); ASSERT(fp);

    fwrite(&n, sizeof(ru64),    1, fp);
    fwrite(d,  sizeof(float)*n,     1, fp);
    fclose(fp);

    return 0;
}

int load_farray(const char *fn, ru64 &n, float **d)
{
    FILE        *fp;
    float       *d_;
    int         i;

    fp = fopen(fn, "r"); ASSERT(fp);

    i = fread(&n, sizeof(ru64), 1, fp);

    d_ = new float[n];
    i = fread(d_, sizeof(float)*n,  1, fp);
    fclose(fp);

    *d = d_;

    return 0;
}



int save_darray(const char *fn, ru64 n, ru64 m, double *d)
{
    FILE        *fp;

    fp = fopen(fn, "w"); ASSERT(fp);

    fwrite(&n, sizeof(ru64),    1, fp);
    fwrite(&m, sizeof(ru64),    1, fp);
    fwrite(d,  sizeof(double)*n*m,  1, fp);
    fclose(fp);

    return 0;
}

int load_darray(const char *fn, ru64 &n, ru64 &m, double **d)
{
    FILE        *fp;
    double      *d_;
    int         i;

    fp = fopen(fn, "r"); ASSERT(fp);

    i = fread(&n, sizeof(ru64), 1, fp);
    i = fread(&m, sizeof(ru64), 1, fp);

    d_ = new double[n*m];
    i = fread(d_, sizeof(double)*n*m, 1, fp);
    fclose(fp);

    *d = d_;

    return 0;
}

int save_farray(const char *fn, ru64 n, ru64 m, float *d)
{
    FILE        *fp;

    fp = fopen(fn, "w"); ASSERT(fp);

    fwrite(&n, sizeof(ru64),    1, fp);
    fwrite(&m, sizeof(ru64),    1, fp);
    fwrite(d,  sizeof(float)*n*m,   1, fp);
    fclose(fp);

    return 0;
}

int load_farray(const char *fn, ru64 &n, ru64 &m, float **d)
{
    FILE        *fp;
    float       *d_;
    int         i;

    fp = fopen(fn, "r"); ASSERT(fp);

    i = fread(&n, sizeof(ru64), 1, fp);
    i = fread(&m, sizeof(ru64), 1, fp);

    d_ = new float[n*m];
    i = fread(d_, sizeof(float)*n*m,  1, fp);
    fclose(fp);

    *d = d_;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// memory function
////////////////////////////////////////////////////////////////////////////////
void memcpy_fast(void *dst, void *src, ru32 s)
{
    memcpy(dst,src,s);
}

void memcpy_fast_(void *dst, void *src, ru32 s)
{
    memcpy(dst,src,s);
}



void conv_argb8888_bgr888(ru8 *src, ru8 *dst, ru32 s)
{
    ru32 i;

    for(i=0; i<s; i++) {
        dst[0] = src[2];
        dst[1] = src[1];
        dst[2] = src[0];

        src += 4;
        dst += 3;
    }
}

// needs:
// src is 16-byte aligned
void conv_argb8888_bgr888_fast(ru8 *src, ru8 *dst, ru32 s)
{
    conv_argb8888_bgr888(src,dst,s);
}


void conv_bgr888_argb8888(ru8 *src, ru8 *dst, ru32 s)
{
    ru32 i;

    for(i=0; i<s; i++) {
        dst[0] = 255;
        dst[1] = src[2];
        dst[2] = src[1];
        dst[3] = src[0];

        src += 3;
        dst += 4;
    }
}


void conv_bgr888_argb8888_fast(ru8 *src, ru8 *dst, ru32 s)
{
    conv_bgr888_argb8888(src,dst,s);
}


void conv_rgb888_bgr888(ru8 *src, ru8 *dst, ru32 s)
{
    ru32 i;
    for(i=0; i<s; i++) {
        dst[0] = src[2];
        dst[1] = src[1];
        dst[2] = src[0];

        src += 3;
        dst += 3;
    }
}

/* in and out must be 16-byte aligned */
// FIXME: copied data error
void conv_rgb888_bgr888_fast(ru8 *in, ru8 *out, ru32 s)
{
    conv_rgb888_bgr888(in,out,s);
}


////////////////////////////////////////////////////////////////////////////////
/// Color pallete
////////////////////////////////////////////////////////////////////////////////

#include "color_table.h"

void get_pal_color(int pal, ru8 v, ru8 *r, ru8 *g, ru8 *b)
{
    *r = Cr[pal*256+v];
    *g = Cg[pal*256+v];
    *b = Cb[pal*256+v];
}



////////////////////////////////////////////////////////////////////////////////
/// Run program
////////////////////////////////////////////////////////////////////////////////

///
/// \brief run a given program
///
/// \param cmd              - program and arguments
///
/// \return
///         0               - success
///
int ExecProgram::run(const std::string &cmd)
{
    m_cmd = cmd;

    return start();
}

///
/// \brief stop the program
///     FIXME: only support Linux
///
/// \return
///         0               - success
///
int ExecProgram::stop(void)
{
    if( !getAlive() ) return 0;

    StringArray sa;
    sa = split_text(m_cmd, " ");

    if( sa.size() > 0 ) {
        string cmd = fmt::sprintf("killall %s", sa[0]);
        system(cmd.c_str());
    }

    return 0;
}

int ExecProgram::thread_func(void *arg)
{
    return system(m_cmd.c_str());
}




////////////////////////////////////////////////////////////////////////////////
/// test module functions
////////////////////////////////////////////////////////////////////////////////

int test_default(CParamArray *pa)
{
    printf("default test routine\n");
    return 0;
}

void print_basic_help(int argc, char *argv[],
                          TestFunctionArray fa[],
                          CParamArray &pa)
{
    int     i, j, k;
    int     tab = 30;

    fmt::print("\n");
    fmt::print("-------------------------- ");
    fmt::print_colored(fmt::GREEN, "basic usage");
    fmt::print(" --------------------------\n");

    fmt::print_colored(fmt::BLUE, "    -f"); fmt::print("              config file\n");
    fmt::print_colored(fmt::BLUE, "    -h"); fmt::print("              print usage help\n");
    fmt::print_colored(fmt::BLUE, "    -dbg_level"); fmt::print("      [0/1/2/3/4/5] debug level\n");
    fmt::print("                        1 - Error\n");
    fmt::print("                        2 - Warning\n");
    fmt::print("                        3 - Info\n");
    fmt::print("                        4 - Trace\n");
    fmt::print("                        5 - Normal\n");
    fmt::print_colored(fmt::BLUE, "    -act"); fmt::print("            [s] test module name\n");

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");

    fmt::print("---------------------------- ");
    fmt::print_colored(fmt::GREEN, "modules");
    fmt::print(" ----------------------------\n");

    i=0;
    while( fa[i].f != NULL ) {
        fmt::print_colored(fmt::BLUE, "    {}", fa[i].name);

        j = strlen(fa[i].name);
        for(k=j; k<tab; k++) fmt::print(" ");

        fmt::print(" : {}\n", fa[i].desc);

        i++;
    }

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");
}


int test_main(int argc, char *argv[],
              TestFunctionArray fa[], CParamArray &pa)
{
    string          act;
    int             dbg_level = 4;
    int             i, j=0;
    pi::ru64        t1=0, t2=0;
    int             ret=-1;
    string          fname;
    StringArray     sa1, sa2;

    // set signal handler
    dbg_stacktrace_setup();

    // parse input arguments
    if( argc <= 1 ) {
        ret = 0;
        goto RTK_TEST_MAIN_PRINT;
    }

    for(i=0; i<argc; i++) {
        // get config file
        if( strcmp(argv[i], "-f") == 0 ) {
            fname = argv[++i];
        }
        // print usage
        else if( strcmp(argv[i], "-h") == 0 ) {
            ret = 0;
            goto RTK_TEST_MAIN_PRINT;
        }
        // debug level
        else if( strcmp(argv[i], "-dbg_level") == 0 ) {
            dbg_level = atoi(argv[++i]);
            dbg_set_level(dbg_level);
        }
    }

    // load config file
    if( fname.length() > 0 )
        pa.load(fname + ".ini");

    // parse input argument again
    pa.set_args(argc, argv);

    // save input arguments to file
    sa1 = path_split(argv[0]);
    sa2 = path_splitext(sa1[1]);
    save_arguments(argc, argv, sa2[0]);

    // print all settings
    pa.print();

    // test actions
    act = "test_default";
    pa.s("act", act);

    // call test function
    i = 0; j = 0;
    while( fa[i].f != NULL ) {
        if( strcmp(act.c_str(), "test_default") == 0 ) {
            test_default(&pa);
            break;
        } else if( strcmp(act.c_str(), fa[i].name) == 0 ) {

            // run routine
            t1 = tm_get_us();
            ret = fa[i].f(&pa);
            t2 = tm_get_us();
            j = 1;

            break;
        }

        i++;
    }

    if( j == 0 ) {
        dbg_pe("Input arguments error!");
        goto RTK_TEST_MAIN_PRINT;
    }

    // print running time
    printf("\n---------------------------------------------------------\n");
    printf("run time = %g sec (%g min)\n",
                1.0*(t2-t1)/1000000.0,         /* sec */
                1.0*(t2-t1)/60000000.0);       /* min */
    printf("---------------------------------------------------------\n");

    goto RTK_TEST_MAIN_RET;

RTK_TEST_MAIN_PRINT:
    // print basic arguments
    print_basic_help(argc, argv, fa, pa);

    // print user provided help
    i=0;
    while( fa[i].f != NULL ) {
        if( strcmp(fa[i].name, "print_help") == 0 ) {
            ret = fa[i].f(&pa);
            break;
        }
        i++;
    }

RTK_TEST_MAIN_RET:
    return ret;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void print_basic_help_ex(int argc, char *argv[],
                         TestModuleArray ma[],
                         CParamArray &pa)
{
    int     i, j, k;
    int     tab = 30;

    fmt::print("\n");
    fmt::print("-------------------------- ");
    fmt::print_colored(fmt::GREEN, "basic usage");
    fmt::print(" --------------------------\n");

    fmt::print_colored(fmt::BLUE, "    -f"); fmt::print("              config file\n");
    fmt::print_colored(fmt::BLUE, "    -h"); fmt::print("              print usage help\n");
    fmt::print_colored(fmt::BLUE, "    -dbg_level"); fmt::print("      [0/1/2/3/4/5] debug level\n");
    fmt::print("                        1 - Error\n");
    fmt::print("                        2 - Warning\n");
    fmt::print("                        3 - Info\n");
    fmt::print("                        4 - Trace\n");
    fmt::print("                        5 - Normal\n");
    fmt::print_colored(fmt::BLUE, "    -act"); fmt::print("            [s] test module name\n");

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");

    fmt::print("---------------------------- ");
    fmt::print_colored(fmt::GREEN, "modules");
    fmt::print(" ----------------------------\n");

    i=0;
    while( ma[i].m != NULL ) {
        fmt::print_colored(fmt::BLUE, "    {}", ma[i].name);

        j = strlen(ma[i].name);
        for(k=j; k<tab; k++) fmt::print(" ");

        fmt::print(" : {}\n", ma[i].desc);

        i++;
    }

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");
}

int test_main(int argc, char *argv[],
              TestModuleArray ma[])
{
    CParamArray     *pa;
    int             isHelp = 0;
    string          act = "";
    int             dbg_level = 4;
    int             i, j=0;
    pi::ru64        t1=0, t2=0;
    int             ret=-1;
    string          fname;
    StringArray     sa1, sa2;

    // set signal handler
    dbg_stacktrace_setup();

    // create a gobal ParameterArray obj
    pa = pa_create();

    // parse input arguments
    if( argc <= 1 ) {
        ret = 0;
        goto RTK_TEST_MAIN_PRINT;
    }

    for(i=0; i<argc; i++) {
        // get config file
        if( strcmp(argv[i], "-f") == 0 ) {
            if( i+1 < argc )
                fname = argv[++i];
            else
                goto RTK_TEST_MAIN_PRINT;
        }
        // print usage
        else if( strcmp(argv[i], "-h") == 0 ) {
            ret = 0;
            isHelp = 1;
        }
        // debug level
        else if( strcmp(argv[i], "-dbg_level") == 0 ) {
            if( i+1 < argc ) {
                dbg_level = atoi(argv[++i]);
                dbg_set_level(dbg_level);
            } else
                goto RTK_TEST_MAIN_PRINT;
        }
        // action
        else if( strcmp(argv[i], "-act") == 0 ) {
            if( i+1 < argc ) {
                act = argv[++i];
            } else
                goto RTK_TEST_MAIN_PRINT;
        }
    }

    if( isHelp ) {
        goto RTK_TEST_MAIN_PRINT;
    }


    // load config file
    if( fname.length() > 0 )
        pa->load(fname + ".ini");

    // set argc, argv to pa
    pa->set_i("argc", argc);
    pa->set_p("argv", argv);

    // parse input argument again
    pa->set_args(argc, argv);

    // save input arguments to file
    sa1 = path_split(argv[0]);
    sa2 = path_splitext(sa1[1]);
    save_arguments(argc, argv, sa2[0]);

    // print all settings
    pa->print();

    // call test function
    i = 0; j = 0;
    while( ma[i].m != NULL ) {
        if( strcmp(act.c_str(), ma[i].name) == 0 ) {

            // run routine
            t1 = tm_get_us();
            ret = ma[i].m->run(pa);
            t2 = tm_get_us();

            j = 1;

            break;
        }

        i++;
    }

    if( j == 0 ) {
        dbg_pe("Input arguments error!");
        goto RTK_TEST_MAIN_PRINT;
    }

    // print running time
    fmt::print("\n---------------------------------------------------------\n");
    fmt::print_colored(fmt::BLUE, "run time");
    fmt::print(" = {} sec ({} min)\n",
                1.0*(t2-t1)/1000000.0,         /* sec */
                1.0*(t2-t1)/60000000.0);       /* min */
    fmt::print("---------------------------------------------------------\n");

    goto RTK_TEST_MAIN_RET;

RTK_TEST_MAIN_PRINT:
    // print basic arguments
    print_basic_help_ex(argc, argv, ma, *pa);

    if( act.size() == 0 ) return ret;

    // call module help function
    i = 0;
    while( ma[i].m != NULL ) {
        if( strcmp(act.c_str(), ma[i].name) == 0 ) {

            fmt::print("\n");
            fmt::print("=======================================\n");
            fmt::print(">> Module: ");
            fmt::print_colored(fmt::BLUE, "{}\n", ma[i].name);
            fmt::print("=======================================\n");

            // run user provided help function
            ma[i].m->help(pa);

            fmt::print("\n");

            break;
        }

        i++;
    }

RTK_TEST_MAIN_RET:
    return ret;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void svar_print_basic_help(int argc, char *argv[],
                          TestFunctionArray fa[])
{
    int     i, j, k;
    int     tab = 30;

    fmt::print("\n");
    fmt::print("-------------------------- ");
    fmt::print_colored(fmt::GREEN, "basic usage");
    fmt::print(" --------------------------\n");

    fmt::print_colored(fmt::BLUE, "    conf=conf_file"); fmt::print("           config file\n");
    fmt::print_colored(fmt::BLUE, "    help"); fmt::print("                     print usage help\n");
    fmt::print_colored(fmt::BLUE, "    dbg_level=<1,2,3,4,5>"); fmt::print("    [0/1/2/3/4/5] debug level\n");
    fmt::print("                                1 - Error\n");
    fmt::print("                                2 - Warning\n");
    fmt::print("                                3 - Info\n");
    fmt::print("                                4 - Trace\n");
    fmt::print("                                5 - Normal\n");
    fmt::print_colored(fmt::BLUE, "    act=act_name"); fmt::print("             [s] test module name\n");

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");

    fmt::print("---------------------------- ");
    fmt::print_colored(fmt::GREEN, "modules");
    fmt::print(" ----------------------------\n");

    i=0;
    while( fa[i].f != NULL ) {
        fmt::print_colored(fmt::BLUE, "    {}", fa[i].name);

        j = strlen(fa[i].name);
        for(k=j; k<tab; k++) fmt::print(" ");

        fmt::print(" : {}\n", fa[i].desc);

        i++;
    }

    fmt::print("-----------------------------------------------------------------\n");
    fmt::print("\n");
}


int svar_main(int argc, char *argv[],
              TestFunctionArray fa[])
{
    string          act, actDefault;
    int             dbg_level = 4;
    int             i, j=0;
    pi::ru64        t1=0, t2=0;
    int             ret=-1;
    string          fn_conf;
    StringArray     sa1, sa2;


    // get program base name
    sa1 = path_splitext(argv[0]);
    fn_conf = sa1[0] + ".cfg";
    sa2 = path_split(sa1[0]);
    actDefault = sa2[1];

    // set argc & argv
    SvarWithType<void*>::instance()["argv"] = argv;
    svar.i["argc"] = argc;

    // parse basic input arguments
    if( argc <= 1 && 0 ) {
        ret = 0;
        goto SVAR_TEST_MAIN_PRINT;
    }

    for(i=1; i<argc; i++) {
        sa1 = split_text(argv[i], "=");

        if( sa1[0] == "conf" ) {
            if( sa1.size() > 1 ) fn_conf = sa1[1];
        } else if( sa1[0] == "help" ) {
            ret = 0;
            goto SVAR_TEST_MAIN_PRINT;
        } else if( sa1[0] == "dbg_level" ) {
            if( sa1.size() > 1 ) {
                dbg_level = atoi(sa1[1].c_str());
                dbg_set_level(dbg_level);
            }
        }
    }

    // load config file
    if( fn_conf.length() > 0 )
        svar.ParseFile(fn_conf);

    // parse input arguments
    for(i=1; i<argc; i++) {
        sa1 = split_text(argv[i], "=");

        if( sa1.size() > 1 ) {
            svar.setvar(argv[i]);
        }
    }

    // save input arguments to file
    if( svar.GetInt("Svar.saveArguments", 0) ) {
        sa2 = path_splitext(argv[0]);
        save_arguments(argc, argv, sa2[0]);
    }

    // print all settings
    svar.dumpAllVars();

    // get action name
    act = svar.GetString("act", actDefault);

    // call corresponding function
    i = 0; j = 0;
    while( fa[i].f != NULL ) {
        if( strcmp(act.c_str(), "test_default") == 0 ) {
            test_default(NULL);
            break;
        } else if( strcmp(act.c_str(), fa[i].name) == 0 ) {

            // run given routine
            t1 = tm_get_us();
            ret = fa[i].f(NULL);
            t2 = tm_get_us();
            j = 1;

            break;
        }

        i++;
    }

    if( j == 0 ) {
        dbg_pe("Input arguments error!");
        goto SVAR_TEST_MAIN_PRINT;
    }

    // print running time
    printf("\n---------------------------------------------------------\n");
    printf("run time = %g sec (%g min)\n",
                1.0*(t2-t1)/1000000.0,         /* sec */
                1.0*(t2-t1)/60000000.0);       /* min */
    printf("---------------------------------------------------------\n");

    goto SVAR_TEST_MAIN_RET;

SVAR_TEST_MAIN_PRINT:
    // print basic arguments
    svar_print_basic_help(argc, argv, fa);

    // print user provided help
    i=0;
    while( fa[i].f != NULL ) {
        if( strcmp(fa[i].name, "print_help") == 0 ) {
            ret = fa[i].f(NULL);
            break;
        }
        i++;
    }

SVAR_TEST_MAIN_RET:
    return ret;
}


} // end of namespace pi
