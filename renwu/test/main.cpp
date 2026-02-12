#include <QtTest/QtTest>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QStringList>
#include <QSet>

// 测试头文件
#include "testmapcache.h"
#include "testmapconverter.h"
#include "testmapwidget.h"
#include "testintegration.h"
#include "testsystem.h"
#include "testmaploader.h"
#include "testnavigationactionclient.h"
#include "testnavigationactionthread.h"
#include "testpathvisualizer.h"
#include "testnavigationintegration.h"
#include "testuser.h"
#include "testuserstorageengine.h"
#include "testuserauthmanager.h"
#include "testlogindialog.h"
#include "testusermanagementdialog.h"
#include "testlogstorageengine.h"
#include "testlogtablemodel.h"
#include "testlogfilterproxymodel.h"
#include "testlogquerytask.h"
#include "testlogthread.h"
#include "testbasethread.h"
#include "testroscontextmanager.h"
#include "testthreadsafequeue.h"
#include "testrobotstatusthread.h"
#include "testnavstatusthread.h"
#include "testsystemmonitorthread.h"
#include "testmapmarker.h"
#include "teststatusindicator.h"
#include "testmapthread.h"
#include "testmainwindow.h"
#include "testnav2parameterthread.h"
#include "testnav2parameterthreadintegration.h"
#include "testnav2viewwidget.h"
#include "testhistorylogmodel.h"

// 测试描述结构
struct TestDescriptor {
    QString name;
    QString category;
    std::function<int(int, char**)> run;
};

// 测试运行器宏
#define RUN_TEST(TestClass) [](int argc, char** argv) -> int { \
    TestClass test; \
    return QTest::qExec(&test, argc, argv); \
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    QCommandLineParser parser;
    parser.setApplicationDescription("Renwu 测试运行器");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption testsOption(QStringList() << "t" << "tests",
                                   "运行指定的测试（逗号分隔，可用: all, log, map, nav, user）",
                                   "tests", "all");
    parser.addOption(testsOption);

    QCommandLineOption listOption(QStringList() << "l" << "list",
                                  "列出所有可用的测试");
    parser.addOption(listOption);

    parser.process(app);

    // 所有可用的测试
    QVector<TestDescriptor> allTests = {
        // Map 相关测试
        {"mapcache", "map", RUN_TEST(TestMapCache)},
        {"mapconverter", "map", RUN_TEST(TestMapConverter)},
        {"mapwidget", "map", RUN_TEST(TestMapWidget)},
        {"maploader", "map", RUN_TEST(TestMapLoader)},
        {"mapmarker", "map", RUN_TEST(TestMapMarker)},
        {"mapthread", "map", RUN_TEST(TestMapThread)},

        // 导航相关测试
        {"navactionclient", "nav", RUN_TEST(TestNavigationActionClient)},
        {"navactionthread", "nav", RUN_TEST(TestNavigationActionThread)},
        {"pathvisualizer", "nav", RUN_TEST(TestPathVisualizer)},
        {"navintegration", "nav", RUN_TEST(TestNavigationIntegration)},
        {"nav2param", "nav", RUN_TEST(TestNav2ParameterThread)},
        {"nav2paramint", "nav", RUN_TEST(TestNav2ParameterThreadIntegration)},
        {"nav2view", "nav", RUN_TEST(TestNav2ViewWidget)},

        // 日志相关测试
        {"logstorage", "log", RUN_TEST(TestLogStorageEngine)},
        {"logtable", "log", RUN_TEST(TestLogTableModel)},
        {"logfilter", "log", RUN_TEST(TestLogFilterProxyModel)},
        {"logquery", "log", RUN_TEST(TestLogQueryTask)},
        {"logthread", "log", RUN_TEST(TestLogThread)},
        {"historylogmodel", "log", RUN_TEST(TestHistoryLogTableModel)},

        // 用户相关测试
        {"user", "user", RUN_TEST(TestUser)},
        {"userstorage", "user", RUN_TEST(TestUserStorageEngine)},
        {"userauth", "user", RUN_TEST(TestUserAuthManager)},
        {"logindialog", "user", RUN_TEST(TestLoginDialog)},
        {"usermgmt", "user", RUN_TEST(TestUserManagementDialog)},

        // 线程和基础组件测试
        {"basethread", "base", RUN_TEST(TestBaseThread)},
        {"roscontext", "base", RUN_TEST(TestROSContextManager)},
        {"threadsafequeue", "base", RUN_TEST(TestThreadSafeQueue)},

        // 状态线程测试
        {"robotstatus", "status", RUN_TEST(TestRobotStatusThread)},
        {"navstatus", "status", RUN_TEST(TestNavStatusThread)},
        {"sysmonitor", "status", RUN_TEST(TestSystemMonitorThread)},
        {"statusindicator", "status", RUN_TEST(TestStatusIndicator)},

        // 其他测试
        {"integration", "other", RUN_TEST(TestIntegration)},
        {"system", "other", RUN_TEST(TestSystem)},
        {"mainwindow", "other", RUN_TEST(TestMainWindow)},
    };

    // 列出所有测试
    if (parser.isSet(listOption)) {
        printf("可用的测试:\n");
        printf("  类别        测试名称        描述\n");
        printf("  ----------------------------------------\n");

        QSet<QString> categories;
        for (const auto& test : allTests) {
            categories.insert(test.category);
        }

        for (const QString& cat : categories) {
            printf("\n[%s]\n", cat.toUtf8().constData());
            for (const auto& test : allTests) {
                if (test.category == cat) {
                    printf("  %s\n", test.name.toUtf8().constData());
                }
            }
        }

        printf("\n快捷方式:\n");
        printf("  all    - 运行所有测试\n");
        printf("  log    - 运行所有日志相关测试\n");
        printf("  map    - 运行所有地图相关测试\n");
        printf("  nav    - 运行所有导航相关测试\n");
        printf("  user   - 运行所有用户相关测试\n");
        printf("  status - 运行所有状态线程测试\n");
        printf("  base   - 运行所有基础组件测试\n");
        printf("\n示例:\n");
        printf("  renwu_tests --tests logstorage,logtable\n");
        printf("  renwu_tests -t log\n");
        printf("  renwu_tests -t all\n");

        return 0;
    }

    // 解析要运行的测试
    QString testsArg = parser.value(testsOption).toLower();
    QSet<QString> testsToRun;

    if (testsArg == "all") {
        for (const auto& test : allTests) {
            testsToRun.insert(test.name);
        }
    } else {
        QStringList parts = testsArg.split(',', Qt::SkipEmptyParts);

        for (const QString& part : parts) {
            QString trimmed = part.trimmed();

            // 快捷方式
            if (trimmed == "log") {
                for (const auto& test : allTests) {
                    if (test.category == "log") {
                        testsToRun.insert(test.name);
                    }
                }
            } else if (trimmed == "map") {
                for (const auto& test : allTests) {
                    if (test.category == "map") {
                        testsToRun.insert(test.name);
                    }
                }
            } else if (trimmed == "nav") {
                for (const auto& test : allTests) {
                    if (test.category == "nav") {
                        testsToRun.insert(test.name);
                    }
                }
            } else if (trimmed == "user") {
                for (const auto& test : allTests) {
                    if (test.category == "user") {
                        testsToRun.insert(test.name);
                    }
                }
            } else if (trimmed == "status") {
                for (const auto& test : allTests) {
                    if (test.category == "status") {
                        testsToRun.insert(test.name);
                    }
                }
            } else if (trimmed == "base") {
                for (const auto& test : allTests) {
                    if (test.category == "base") {
                        testsToRun.insert(test.name);
                    }
                }
            } else {
                // 直接匹配测试名称
                bool found = false;
                for (const auto& test : allTests) {
                    if (test.name.startsWith(trimmed)) {
                        testsToRun.insert(test.name);
                        found = true;
                    }
                }
                if (!found) {
                    fprintf(stderr, "警告: 未找到测试 '%s'\n", trimmed.toUtf8().constData());
                }
            }
        }
    }

    // 如果没有指定任何测试，显示帮助
    if (testsToRun.isEmpty()) {
        fprintf(stderr, "错误: 没有指定要运行的测试\n");
        fprintf(stderr, "使用 --list 查看所有可用的测试\n");
        return 1;
    }

    // 运行选定的测试
    int result = 0;
    int passed = 0;
    int failed = 0;

    printf("将运行 %d 个测试:\n", testsToRun.size());
    for (const QString& testName : testsToRun) {
        printf("  - %s\n", testName.toUtf8().constData());
    }
    printf("\n");

    // 准备传递给 QTest 的参数（只保留 QTest 相关的参数）
    QStringList qtestArgs;
    qtestArgs << argv[0];

    // Qt 测试参数列表
    QSet<QString> qtestOptions = {
        "-o", "-vs", "-v1", "-v2", "-v3", "-silent",
        "-functions", "-datatags", "-eventdelay", "-keydelay",
        "-mousedelay", "-keydelay", "-maxwarnings",
        "-help", "-h", "-version"
    };

    bool skipNext = false;
    for (int i = 1; i < argc; ++i) {
        if (skipNext) {
            skipNext = false;
            continue;
        }

        QString arg = QString::fromLocal8Bit(argv[i]);

        // 跳过我们自定义的参数
        if (arg == "-t" || arg == "--tests" || arg == "-l" || arg == "--list") {
            if (arg == "-t" || arg == "--tests" || arg == "-l") {
                skipNext = true;  // 跳过参数值
            }
            continue;
        }

        // 只保留 Qt 测试相关的参数
        if (arg.startsWith("-")) {
            if (qtestOptions.contains(arg) || arg.startsWith("-o:")) {
                qtestArgs << arg;
            }
        } else {
            // 非选项参数，可能是 Qt 测试的测试函数名称
            // 但我们不需要它，因为我们在代码中指定了测试类
        }
    }

    // 将 QStringList 转换为 char* 数组
    QVector<char*> qtestArgv;
    QVector<QByteArray> qtestArgBytes;
    for (const QString& arg : qtestArgs) {
        qtestArgBytes.append(arg.toLocal8Bit());
        qtestArgv.append(qtestArgBytes.last().data());
    }

    int qtestArgc = qtestArgv.size();

    for (const auto& test : allTests) {
        if (testsToRun.contains(test.name)) {
            printf("********* 运行 %s *********\n", test.name.toUtf8().constData());
            int testResult = test.run(qtestArgc, qtestArgv.data());
            if (testResult == 0) {
                passed++;
                printf("PASS: %s\n", test.name.toUtf8().constData());
            } else {
                failed++;
                printf("FAIL: %s (退出码: %d)\n", test.name.toUtf8().constData(), testResult);
            }
            result |= testResult;
            printf("\n");
        }
    }

    // 打印总结
    printf("==================== 测试总结 ====================\n");
    printf("总计: %d, 通过: %d, 失败: %d\n", passed + failed, passed, failed);
    printf("=================================================\n");

    return result;
}
