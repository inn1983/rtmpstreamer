#ifndef CDX_Debug_h
#define CDX_Debug_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef LOG_NDEBUG
#define LOG_NDEBUG 1
#endif

#if defined LOG_FILE
#define LOGD(fd, ...) ((void)fprintf(fd, "D/" LOG_TAG ": "));         \
	((void)fprintf(fd, "(%d) ", __LINE__));						 \
	((void)fprintf(fd, __VA_ARGS__));						     \
	((void)fprintf(fd, "\n"))

#define LOGE(fd, ...) ((void)fprintf(fd, "E/" LOG_TAG ": "));         \
	((void)fprintf(fd, "(%d) ", __LINE__));						 \
	((void)fprintf(fd, __VA_ARGS__));						     \
	((void)fprintf(fd, "\n"))

#elif defined LOG_FILE_DUMMY

#define LOGD(fd, ...) ((void)printf("D/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))

#define LOGE(fd, ...) ((void)printf("E/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))

#else

#define LOGD(...) ((void)printf("D/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))

#define LOGE(...) ((void)printf("E/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))
#endif	//LOG_FILE

#if LOG_NDEBUG
#define LOGV(...)   ((void)0)
#define LOGI(...)   ((void)0)
#define LOGW(...)   ((void)0)
	
#else
#define LOGV(...) ((void)printf("V/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))
	
#define LOGI(...) ((void)printf("I/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))

#define LOGW(...) ((void)printf("W/" LOG_TAG ": "));         \
	((void)printf("(%d) ", __LINE__));						 \
	((void)printf(__VA_ARGS__));						     \
	((void)printf("\n"))

#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
