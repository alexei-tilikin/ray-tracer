
#ifndef ARGUMENTS_HH_
#define ARGUMENTS_HH_

#include <cstdlib>
#include <cstring>
#include <string>

#include "defs.hh"

using std::string;

/* Holder class for parsed cmd arguments.
 * Parses arguments from raw args array.
 * On parsing error: process terminated with code 1. 
 */
class Arguments 
{
public:
    static const string ARGS[];
    
    enum Index {
        ARG_HEIGHT = 0,
        ARG_WIDTH,
        ARG_SUPERSAMPLE,
        ARG_AVG_REFLECTIONS,
        ARG_REFLECT_CUTOFF,
        ARG_RESOURCE_PATH
    };
    
    int height;
    int width;
    int samplesPerPixel;
    int numReflections;
    double reflectionsCutoff;
    string resourcePath;
    
    Arguments(int argc, const char* argv[])
    //here default values for args defined
        : height(800), width(800), 
        samplesPerPixel(1), numReflections(1), reflectionsCutoff(0.0),
        resourcePath("../../"), _argc(argc), _argv(argv)
    {
        
    }
    
    void parse();
#ifndef NDEBUG   
    void print() const 
    {
        std::cout << "height: " << height
        << "\nwidth: " << width
        << "\nresourcePath: " << resourcePath
        << "\nsamplesPerPixel: " << samplesPerPixel
        << "\nnumReflections: " << numReflections
        << "\nreflectionsCutoff: " << reflectionsCutoff
        << std::endl;
    }
#else
    void print() const {};
#endif //NDEBUG
    
private:
    const int _argc;
    const char** _argv; 
    
    void verifyHasArg(int i, int argIdx);
};

inline static int parseInt(const char *arg) {
    return (int) strtol(arg, NULL, 10);
}

inline static double parseDouble(const char *arg) {
    return strtod(arg, NULL);
}

inline static void parseString(const char *arg, string &dest) {
    static char buff[128];
    size_t termChar = std::min<size_t>(127, strlen(arg));
    strncpy(buff, arg, termChar);
    buff[termChar] = '\0';
    dest = string(buff);
}

//code patterns: for Arguments::parse() only

#define ARG_SELECT(name, target, parser, lower, upper) \
if (ARGS[name] == currArg) { \
verifyHasArg(++i, name); \
parser(name, target, lower, upper) \
}

#define INT_ARG(name, target, lower, upper) \
target = std::min(upper, std::max(lower, parseInt(_argv[i])));

#define DOUBLE_ARG(name, target, lower, upper) \
target = std::min(upper, std::max(lower, parseDouble(_argv[i])));

#define ANGLE_ARG(name, target, lower, upper) \
DOUBLE_ARG(name, target, lower, upper) \
target = deg2rad(target); \

#define STRING_ARG(name, target, lower, upper) \
parseString(_argv[i], target);

inline void Arguments::parse()
{
    string currArg;
    for (int i = 1; i < _argc; ++i)
    {
        parseString(_argv[i], currArg);
        ARG_SELECT(ARG_HEIGHT, height, INT_ARG, 50, 1000)
        else ARG_SELECT(ARG_WIDTH, width, INT_ARG, 50, 1000)
        else ARG_SELECT(ARG_SUPERSAMPLE, samplesPerPixel, INT_ARG, 1, 100)
        else ARG_SELECT(ARG_AVG_REFLECTIONS, numReflections, INT_ARG, 1, 100)
        else ARG_SELECT(ARG_REFLECT_CUTOFF, reflectionsCutoff, ANGLE_ARG, 0.0, 89.0)
        else ARG_SELECT(ARG_RESOURCE_PATH, resourcePath, STRING_ARG, "", "")
        else 
        {
            cerr << "Illegal argument name: " << currArg << endl;
            exit(1);
        }
    }
}

#undef ARG_SELECT
#undef INT_ARG
#undef DOUBLE_ARG
#undef ANGLE_ARG
#undef STRING_ARG

inline void Arguments::verifyHasArg(int i, int argIdx) {
    if (i >= _argc)
    {
        cerr << "Expecting argument value after " << ARGS[argIdx] << endl;
        exit(1);
    }
}

#endif /* ARGUMENTS_HH_ */
