import datetime



class Logger(object):

    COLORS = {  # Unix codes for special priting
        "HEADER"	: 	{"value" :'\033[95m', "code" :0},
        "OKBLUE" 		: 	{"value" :'\033[94m', "code" :1},
        "OKGREEN" 		: 	{"value" :'\033[92m', "code" :2},
        "WARNING" 		: 	{"value" :'\033[93m', "code" :3},
        "FAIL" 			: 	{"value" :'\033[91m', "code" :4},
        "ENDC" 			: 	{"value" :'\033[0m', "code" :5},
        "BOLD" 			: 	{"value" :'\033[1m', "code" :6},
        "UNDERLINE" 	: 	{"value" :'\033[4m', "code" :7},
        "STANDARD"		:	{"value" :'', "code" :8}
    }

    @staticmethod
    def log( caller, message, color):

        # [03/Apr/2017 18:37:10]
        time = datetime.datetime.now()

        if color not in Logger.COLORS:
            color = Logger.COLORS.get("STANDARD").get("value")

        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
        prefix = prefix +  Logger.COLORS["BOLD"]["value"]+ Logger.COLORS["UNDERLINE"]["value"]+ caller+ Logger.COLORS["ENDC"]["value"]+ ":"
        print_Str = prefix + " "+Logger.COLORS[color]["value"] + " "+message + " "+Logger.COLORS["ENDC"]["value"]

        print(print_Str)


    @staticmethod
    def pp_list(list):
        ret = "[ "
        for i in list:
            ret += "%.5f" % i
            ret += ", "
        ret = ret[:-2]
        ret += "] "
        return ret

    @staticmethod
    def pp_double(dbl):
        s = "%.5f" % dbl
        return s