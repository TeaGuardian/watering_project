from datetime import datetime, timedelta
from PyQt5.QtWidgets import QWidget, QTabWidget, QProgressBar, QPushButton, QLabel, QFrame, QLineEdit
from PyQt5.QtWidgets import QLCDNumber, QComboBox, QCheckBox, QSpinBox, QMainWindow, QApplication
from PyQt5.QtWidgets import QTextBrowser, QAbstractScrollArea, QTextEdit
from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot, QMetaObject, QCoreApplication, QTimer, Qt, QIODevice
from PyQt5.QtGui import QPixmap
from PyQt5 import QtSerialPort


class MainWindow(object):
    def setupUi(self, Form):
        Form.setObjectName("ММАПУТ control panel")
        Form.resize(638, 488)
        Form.setStyleSheet("background-color: rgb(104, 104, 104);\nfont: 11pt \"GOST type B\";")
        self.tabWidget = QTabWidget(Form)
        self.tabWidget.setEnabled(True)
        self.tabWidget.setGeometry(0, 0, 641, 491)
        self.tabWidget.setStyleSheet("background-color: rgb(83, 83, 83);")
        self.tabWidget.setDocumentMode(False)
        self.tabWidget.setObjectName("tabWidget")
        self.connect = QWidget()
        self.connect.setObjectName("connect")
        self.com = QComboBox(self.connect)
        self.com.setGeometry(10, 50, 361, 21)
        self.com.setStyleSheet("background-color: rgb(240, 255, 241);\nselection-color: rgb(220, 239, 255);\ncolor: rgb(61, 77, 62);")
        self.com.setObjectName("com")
        self.label = QLabel(self.connect)
        self.label.setGeometry(10, 5, 611, 31)
        self.label.setStyleSheet("color: rgb(229, 255, 235);")
        self.label.setFrameShape(QFrame.StyledPanel)
        self.label.setLineWidth(2)
        self.label.setScaledContents(False)
        self.label.setAlignment(Qt.AlignBottom|Qt.AlignHCenter)
        self.label.setWordWrap(False)
        self.label.setObjectName("label")
        self.upd_com = QPushButton(self.connect)
        self.upd_com.setGeometry(380, 50, 91, 23)
        self.upd_com.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.upd_com.setObjectName("upd_com")
        self.con_com = QPushButton(self.connect)
        self.con_com.setGeometry(480, 50, 121, 23)
        self.con_com.setStyleSheet("background-color: rgb(215, 255, 237);")
        self.con_com.setObjectName("con_com")
        self.upd_pb = QProgressBar(self.connect)
        self.upd_pb.setGeometry(10, 93, 611, 16)
        self.upd_pb.setAutoFillBackground(False)
        self.upd_pb.setMinimum(0)
        self.upd_pb.setProperty("value", 0)
        self.upd_pb.setTextVisible(False)
        self.upd_pb.setInvertedAppearance(False)
        self.upd_pb.setObjectName("upd_pb")
        self.line = QFrame(self.connect)
        self.line.setGeometry(10, 125, 611, 21)
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)
        self.line.setObjectName("line")
        self.index_fl = QCheckBox(self.connect)
        self.index_fl.setGeometry(9, 150, 111, 21)
        self.index_fl.setAccessibleName("")
        self.index_fl.setStyleSheet("color: rgb(226, 255, 252);")
        self.index_fl.setObjectName("index_fl")
        self.sett_fl = QCheckBox(self.connect)
        self.sett_fl.setGeometry(150, 150, 161, 17)
        self.sett_fl.setStyleSheet("color: rgb(226, 255, 252);")
        self.sett_fl.setCheckable(False)
        self.sett_fl.setChecked(False)
        self.sett_fl.setTristate(False)
        self.sett_fl.setObjectName("sett_fl")
        self.ign_fl = QCheckBox(self.connect)
        self.ign_fl.setGeometry(340, 150, 161, 17)
        self.ign_fl.setStyleSheet("color: rgb(226, 255, 252);")
        self.ign_fl.setCheckable(True)
        self.ign_fl.setChecked(True)
        self.ign_fl.setObjectName("ign_fl")
        self.textBrowser = QTextBrowser(self.connect)
        self.textBrowser.setGeometry(10, 280, 611, 171)
        self.textBrowser.setAutoFillBackground(False)
        self.textBrowser.setStyleSheet("background-color: rgb(81, 81, 81);\ncolor: rgb(218, 236, 228);")
        self.textBrowser.setFrameShape(QFrame.Box)
        self.textBrowser.setFrameShadow(QFrame.Plain)
        self.textBrowser.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.textBrowser.setObjectName("textBrowser")
        self.tabWidget.addTab(self.connect, "")
        self.control = QWidget()
        self.control.setObjectName("control")
        self.comboBox_2 = QComboBox(self.control)
        self.comboBox_2.setGeometry(10, 10, 271, 21)
        self.comboBox_2.setStyleSheet("background-color: rgb(240, 255, 241);\nselection-color: rgb(220, 239, 255);\ncolor: rgb(61, 77, 62);")
        self.comboBox_2.setObjectName("comboBox_2")
        self.update2 = QPushButton(self.control)
        self.update2.setGeometry(300, 10, 91, 23)
        self.update2.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.update2.setObjectName("update2")
        self.switch1 = QPushButton(self.control)
        self.switch1.setGeometry(120, 50, 101, 31)
        self.switch1.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.switch1.setObjectName("switch1")
        self.label_2 = QLabel(self.control)
        self.label_2.setGeometry(10, 50, 101, 31)
        self.label_2.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_2.setFrameShape(QFrame.Box)
        self.label_2.setLineWidth(2)
        self.label_2.setScaledContents(False)
        self.label_2.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_2.setWordWrap(False)
        self.label_2.setObjectName("label_2")
        self.label_3 = QLabel(self.control)
        self.label_3.setGeometry(230, 50, 31, 31)
        self.label_3.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_3.setFrameShape(QFrame.Box)
        self.label_3.setLineWidth(2)
        self.label_3.setScaledContents(False)
        self.label_3.setAlignment(Qt.AlignCenter)
        self.label_3.setWordWrap(False)
        self.label_3.setObjectName("label_3")
        self.label_4 = QLabel(self.control)
        self.label_4.setGeometry(10, 90, 101, 31)
        self.label_4.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_4.setFrameShape(QFrame.Box)
        self.label_4.setLineWidth(2)
        self.label_4.setScaledContents(False)
        self.label_4.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_4.setWordWrap(False)
        self.label_4.setObjectName("label_4")
        self.switch2 = QPushButton(self.control)
        self.switch2.setGeometry(120, 90, 101, 31)
        self.switch2.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.switch2.setObjectName("switch2")
        self.label_5 = QLabel(self.control)
        self.label_5.setGeometry(230, 90, 31, 31)
        self.label_5.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_5.setFrameShape(QFrame.Box)
        self.label_5.setLineWidth(2)
        self.label_5.setScaledContents(False)
        self.label_5.setAlignment(Qt.AlignCenter)
        self.label_5.setWordWrap(False)
        self.label_5.setObjectName("label_5")
        self.label_6 = QLabel(self.control)
        self.label_6.setGeometry(10, 130, 81, 31)
        self.label_6.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_6.setFrameShape(QFrame.Box)
        self.label_6.setLineWidth(2)
        self.label_6.setScaledContents(False)
        self.label_6.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_6.setWordWrap(False)
        self.label_6.setObjectName("label_6")
        self.led_br = QSpinBox(self.control)
        self.led_br.setGeometry(110, 130, 81, 31)
        self.led_br.setStyleSheet("background-color: rgb(233, 255, 251);")
        self.led_br.setMaximum(255)
        self.led_br.setSingleStep(10)
        self.led_br.setObjectName("spinBox")
        self.led_but = QPushButton(self.control)
        self.led_but.setGeometry(200, 130, 61, 31)
        self.led_but.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.led_but.setObjectName("pushButton_6")
        self.buz_time = QSpinBox(self.control)
        self.buz_time.setGeometry(80, 170, 61, 31)
        self.buz_time.setStyleSheet("background-color: rgb(233, 255, 251);")
        self.buz_time.setMaximum(100000)
        self.buz_time.setSingleStep(10)
        self.buz_time.setObjectName("spinBox_2")
        self.buz_but = QPushButton(self.control)
        self.buz_but.setGeometry(230, 170, 31, 31)
        self.buz_but.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.buz_but.setObjectName("pushButton_7")
        self.label_7 = QLabel(self.control)
        self.label_7.setGeometry(10, 170, 61, 31)
        self.label_7.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_7.setFrameShape(QFrame.Box)
        self.label_7.setLineWidth(2)
        self.label_7.setScaledContents(False)
        self.label_7.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_7.setWordWrap(False)
        self.label_7.setObjectName("label_7")
        self.buz_freq = QSpinBox(self.control)
        self.buz_freq.setGeometry(150, 170, 71, 31)
        self.buz_freq.setStyleSheet("background-color: rgb(233, 255, 251);")
        self.buz_freq.setMaximum(100000)
        self.buz_freq.setSingleStep(10)
        self.buz_freq.setObjectName("spinBox_3")
        self.mol1 = QLabel(self.control)
        self.mol1.setGeometry(10, 220, 61, 31)
        self.mol1.setStyleSheet("color: rgb(229, 255, 235);")
        self.mol1.setFrameShape(QFrame.Box)
        self.mol1.setLineWidth(2)
        self.mol1.setScaledContents(False)
        self.mol1.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.mol1.setWordWrap(False)
        self.mol1.setObjectName("mol1")
        self.mol_1_lcd = QLCDNumber(self.control)
        self.mol_1_lcd.setGeometry(90, 220, 121, 31)
        self.mol_1_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.mol_1_lcd.setObjectName("lcdNumber")
        self.mol2 = QLabel(self.control)
        self.mol2.setGeometry(10, 260, 61, 31)
        self.mol2.setStyleSheet("color: rgb(229, 255, 235);")
        self.mol2.setFrameShape(QFrame.Box)
        self.mol2.setLineWidth(2)
        self.mol2.setScaledContents(False)
        self.mol2.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.mol2.setWordWrap(False)
        self.mol2.setObjectName("mol2")
        self.mol_2_lcd = QLCDNumber(self.control)
        self.mol_2_lcd.setGeometry(90, 260, 121, 31)
        self.mol_2_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.mol_2_lcd.setObjectName("lcdNumber_3")
        self.b1 = QLabel(self.control)
        self.b1.setGeometry(10, 300, 61, 31)
        self.b1.setStyleSheet("color: rgb(229, 255, 235);")
        self.b1.setFrameShape(QFrame.Box)
        self.b1.setLineWidth(2)
        self.b1.setScaledContents(False)
        self.b1.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.b1.setWordWrap(False)
        self.b1.setObjectName("b1")
        self.but_1_lcd = QLCDNumber(self.control)
        self.but_1_lcd.setGeometry(90, 300, 121, 31)
        self.but_1_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.but_1_lcd.setObjectName("lcdNumber_4")
        self.b2 = QLabel(self.control)
        self.b2.setGeometry(10, 340, 61, 31)
        self.b2.setStyleSheet("color: rgb(229, 255, 235);")
        self.b2.setFrameShape(QFrame.Box)
        self.b2.setLineWidth(2)
        self.b2.setScaledContents(False)
        self.b2.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.b2.setWordWrap(False)
        self.b2.setObjectName("b2")
        self.but_2_lcd = QLCDNumber(self.control)
        self.but_2_lcd.setGeometry(90, 340, 121, 31)
        self.but_2_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.but_2_lcd.setObjectName("lcdNumber_5")
        self.aqi = QLabel(self.control)
        self.aqi.setGeometry(320, 50, 61, 31)
        self.aqi.setStyleSheet("color: rgb(229, 255, 235);")
        self.aqi.setFrameShape(QFrame.Box)
        self.aqi.setLineWidth(2)
        self.aqi.setScaledContents(False)
        self.aqi.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.aqi.setWordWrap(False)
        self.aqi.setObjectName("aqi")
        self.aqi_lcd = QLCDNumber(self.control)
        self.aqi_lcd.setGeometry(400, 50, 121, 31)
        self.aqi_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.aqi_lcd.setObjectName("lcdNumber_6")
        self.eco2 = QLabel(self.control)
        self.eco2.setGeometry(320, 90, 61, 31)
        self.eco2.setStyleSheet("color: rgb(229, 255, 235);")
        self.eco2.setFrameShape(QFrame.Box)
        self.eco2.setLineWidth(2)
        self.eco2.setScaledContents(False)
        self.eco2.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.eco2.setWordWrap(False)
        self.eco2.setObjectName("eco2")
        self.eco2_lcd = QLCDNumber(self.control)
        self.eco2_lcd.setGeometry(400, 90, 121, 31)
        self.eco2_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.eco2_lcd.setObjectName("lcdNumber_7")
        self.tvoc = QLabel(self.control)
        self.tvoc.setGeometry(320, 130, 61, 31)
        self.tvoc.setStyleSheet("color: rgb(229, 255, 235);")
        self.tvoc.setFrameShape(QFrame.Box)
        self.tvoc.setLineWidth(2)
        self.tvoc.setScaledContents(False)
        self.tvoc.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.tvoc.setWordWrap(False)
        self.tvoc.setObjectName("tvoc")
        self.tvoc_lcd = QLCDNumber(self.control)
        self.tvoc_lcd.setGeometry(400, 130, 121, 31)
        self.tvoc_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.tvoc_lcd.setObjectName("lcdNumber_8")
        self.temp = QLabel(self.control)
        self.temp.setGeometry(320, 170, 61, 31)
        self.temp.setStyleSheet("color: rgb(229, 255, 235);")
        self.temp.setFrameShape(QFrame.Box)
        self.temp.setLineWidth(2)
        self.temp.setScaledContents(False)
        self.temp.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.temp.setWordWrap(False)
        self.temp.setObjectName("temp")
        self.temp_lcd = QLCDNumber(self.control)
        self.temp_lcd.setGeometry(400, 170, 121, 31)
        self.temp_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.temp_lcd.setObjectName("lcdNumber_9")
        self.hum = QLabel(self.control)
        self.hum.setGeometry(320, 210, 61, 31)
        self.hum.setStyleSheet("color: rgb(229, 255, 235);")
        self.hum.setFrameShape(QFrame.Box)
        self.hum.setLineWidth(2)
        self.hum.setScaledContents(False)
        self.hum.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.hum.setWordWrap(False)
        self.hum.setObjectName("hum")
        self.hum_lcd = QLCDNumber(self.control)
        self.hum_lcd.setGeometry(400, 210, 121, 31)
        self.hum_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.hum_lcd.setObjectName("lcdNumber_10")
        self.sv = QLabel(self.control)
        self.sv.setGeometry(320, 250, 61, 31)
        self.sv.setStyleSheet("color: rgb(229, 255, 235);")
        self.sv.setFrameShape(QFrame.Box)
        self.sv.setLineWidth(2)
        self.sv.setScaledContents(False)
        self.sv.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.sv.setWordWrap(False)
        self.sv.setObjectName("sv")
        self.sv_lcd = QLCDNumber(self.control)
        self.sv_lcd.setGeometry(400, 250, 121, 31)
        self.sv_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.sv_lcd.setObjectName("lcdNumber_11")
        self.bv = QLabel(self.control)
        self.bv.setGeometry(320, 290, 61, 31)
        self.bv.setStyleSheet("color: rgb(229, 255, 235);")
        self.bv.setFrameShape(QFrame.Box)
        self.bv.setLineWidth(2)
        self.bv.setScaledContents(False)
        self.bv.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.bv.setWordWrap(False)
        self.bv.setObjectName("bv")
        self.bv_lcd = QLCDNumber(self.control)
        self.bv_lcd.setGeometry(400, 290, 121, 31)
        self.bv_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.bv_lcd.setObjectName("lcdNumber_12")
        self.i_s = QLabel(self.control)
        self.i_s.setGeometry(320, 330, 61, 31)
        self.i_s.setStyleSheet("color: rgb(229, 255, 235);")
        self.i_s.setFrameShape(QFrame.Box)
        self.i_s.setLineWidth(2)
        self.i_s.setScaledContents(False)
        self.i_s.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.i_s.setWordWrap(False)
        self.i_s.setObjectName("i_s")
        self.is_lcd = QLCDNumber(self.control)
        self.is_lcd.setGeometry(400, 330, 121, 31)
        self.is_lcd.setStyleSheet("color: rgb(26, 21, 14);\nbackground-color: rgb(244, 252, 255);")
        self.is_lcd.setObjectName("lcdNumber_13")
        self.progressBar = QProgressBar(self.control)
        self.progressBar.setGeometry(10, 440, 611, 16)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setTextVisible(False)
        self.progressBar.setObjectName("progressBar")
        self.tabWidget.addTab(self.control, "")
        self.command = QWidget()
        self.command.setObjectName("command")
        self.output = QTextEdit(self.command)
        self.output.setGeometry(10, 50, 611, 401)
        self.output.setStyleSheet("background-color: rgb(208, 225, 220);")
        self.output.setDocumentTitle("")
        self.output.setReadOnly(True)
        self.output.setObjectName("output")
        self.clear = QPushButton(self.command)
        self.clear.setGeometry(530, 10, 91, 31)
        self.clear.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.clear.setObjectName("clear")
        self.send = QPushButton(self.command)
        self.send.setGeometry(430, 10, 91, 31)
        self.send.setStyleSheet("background-color: rgb(239, 255, 243);")
        self.send.setObjectName("send")
        self.lineEdit = QLineEdit(self.command)
        self.lineEdit.setGeometry(10, 10, 411, 31)
        self.lineEdit.setStyleSheet("background-color: rgb(242, 255, 239);\ncolor: rgb(62, 62, 62);")
        self.lineEdit.setText("")
        self.lineEdit.setMaxLength(1000)
        self.lineEdit.setObjectName("lineEdit")
        self.tabWidget.addTab(self.command, "")
        self.auto = QWidget()
        self.auto.setObjectName("auto")
        self.textBrowser_2 = QTextBrowser(self.auto)
        self.textBrowser_2.setGeometry(10, 10, 611, 51)
        self.textBrowser_2.setAutoFillBackground(False)
        self.textBrowser_2.setStyleSheet("background-color: rgb(81, 81, 81);\ncolor: rgb(218, 236, 228);")
        self.textBrowser_2.setFrameShape(QFrame.Box)
        self.textBrowser_2.setFrameShadow(QFrame.Plain)
        self.textBrowser_2.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser_2.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser_2.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.spinBox_4 = QSpinBox(self.auto)
        self.spinBox_4.setGeometry(210, 70, 111, 31)
        self.spinBox_4.setStyleSheet("background-color: rgb(233, 255, 251);")
        self.spinBox_4.setMaximum(1023)
        self.spinBox_4.setSingleStep(10)
        self.spinBox_4.setObjectName("spinBox_4")
        self.label_8 = QLabel(self.auto)
        self.label_8.setGeometry(10, 70, 181, 31)
        self.label_8.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_8.setFrameShape(QFrame.Box)
        self.label_8.setLineWidth(2)
        self.label_8.setScaledContents(False)
        self.label_8.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_8.setWordWrap(False)
        self.label_8.setObjectName("label_8")
        self.spinBox_9 = QSpinBox(self.auto)
        self.spinBox_9.setGeometry(210, 120, 111, 31)
        self.spinBox_9.setStyleSheet("background-color: rgb(233, 255, 251);")
        self.spinBox_9.setMaximum(1023)
        self.spinBox_9.setSingleStep(10)
        self.spinBox_9.setObjectName("spinBox_9")
        self.label_17 = QLabel(self.auto)
        self.label_17.setGeometry(10, 120, 181, 31)
        self.label_17.setStyleSheet("color: rgb(229, 255, 235);")
        self.label_17.setFrameShape(QFrame.Box)
        self.label_17.setLineWidth(2)
        self.label_17.setScaledContents(False)
        self.label_17.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.label_17.setWordWrap(False)
        self.label_17.setObjectName("label_17")
        self.textBrowser_5 = QTextBrowser(self.auto)
        self.textBrowser_5.setGeometry(10, 180, 611, 81)
        self.textBrowser_5.setAutoFillBackground(False)
        self.textBrowser_5.setStyleSheet("background-color: rgb(81, 81, 81);\ncolor: rgb(218, 236, 228);")
        self.textBrowser_5.setFrameShape(QFrame.Box)
        self.textBrowser_5.setFrameShadow(QFrame.Plain)
        self.textBrowser_5.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser_5.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.textBrowser_5.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.textBrowser_5.setObjectName("textBrowser_5")
        self.tabWidget.addTab(self.auto, "")
        self.info = QWidget()
        self.info.setObjectName("info")
        self.qr_code = QLabel(self.info)
        self.qr_code.setGeometry(100, 20, 431, 411)
        self.qr_code.setText("")
        self.qr_code.setObjectName("qr_code")
        self.tabWidget.addTab(self.info, "")
        self.qr_code.setPixmap(QPixmap('qr.png').scaled(self.qr_code.width(), self.qr_code.height()))
        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "ММАПУТ demo control panel"))
        self.connect.setAccessibleName(_translate("Form", "соединение"))
        self.label.setText(_translate("Form", "выберите COM порт вашего переходника"))
        self.upd_com.setText(_translate("Form", "обновить"))
        self.con_com.setText(_translate("Form", "подключиться"))
        self.index_fl.setText(_translate("Form", "индексировать"))
        self.sett_fl.setText(_translate("Form", "сохранять настройки"))
        self.ign_fl.setText(_translate("Form", "автоматика"))
        self.textBrowser.setPlaceholderText(_translate("Form", "Данная версия является демонстрационной и не подлежит использованию в комерческих целях, как и все распространяемые материалы, для комерческого использования свяжитесь с разработчиком (Юсупов Эрик Рустамович). Используя данную версию вы соглашаетесь с условиями использования нашего оборудования, пожалуйста не забывайте обслуживать модули. Данный софт расчитан на модули поколения 0 с кодовым номером 3S2N22 (включающий в себя: 2 силовых ключа для управления клапанами, датчик ens160, датчик aht2x, датчик тока ina219, 2 сенсорные кнопки, управляемый светодиод, управляемый динамик, 2 программных uart интерфейса и индикатор данных) (напряжение работы 12в, потребляемый ток до 500мА, скорость шины данных 9600 бод)."))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.connect), _translate("Form", "подключение"))
        self.control.setAccessibleName(_translate("Form", "управление"))
        self.update2.setText(_translate("Form", "обновить"))
        self.switch1.setText(_translate("Form", "переключить"))
        self.label_2.setText(_translate("Form", "клапан воды"))
        self.label_3.setText(_translate("Form", "?"))
        self.label_4.setText(_translate("Form", "клапан газа"))
        self.switch2.setText(_translate("Form", "переключить"))
        self.label_5.setText(_translate("Form", "?"))
        self.label_6.setText(_translate("Form", "светодиод"))
        self.led_but.setText(_translate("Form", "-->"))
        self.buz_but.setText(_translate("Form", "-->"))
        self.label_7.setText(_translate("Form", "динамик"))
        self.mol1.setText(_translate("Form", "почва 1"))
        self.mol2.setText(_translate("Form", "почва 2"))
        self.b1.setText(_translate("Form", "кнопка 1"))
        self.b2.setText(_translate("Form", "кнопка 2"))
        self.aqi.setText(_translate("Form", "AQI"))
        self.eco2.setText(_translate("Form", "eCO2"))
        self.tvoc.setText(_translate("Form", "TVOC"))
        self.temp.setText(_translate("Form", "Температура"))
        self.hum.setText(_translate("Form", "Влажность"))
        self.sv.setText(_translate("Form", "SV"))
        self.bv.setText(_translate("Form", "BV"))
        self.i_s.setText(_translate("Form", "    I"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.control), _translate("Form", "управление"))
        self.clear.setText(_translate("Form", "ОЧИСТИТЬ"))
        self.send.setText(_translate("Form", "ОТПРАВИТЬ"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.command), _translate("Form", "консоль"))
        self.textBrowser_2.setPlaceholderText(_translate("Form", "значение влажности почвы от 0 до 1023, чем выше тем, суше почва. Контроль углекислого газа не подвязан для того, чтобы вы смогли сконцентрироваться на воде."))
        self.label_8.setText(_translate("Form", "максимальная влажность"))
        self.label_17.setText(_translate("Form", "минимальная влажность"))
        self.textBrowser_5.setPlaceholderText(_translate("Form", "если макс > чем мин, клапан выключен всегда. Список команд для консоли вы можете найти на странице проекта. Данные вводяться через разделитель \'#\', заполняются все 6 полей в таком порядке: id отправителя (0 жеательно), id получателя, флаг команды, номер команды, дробные значения, целые значения"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.auto), _translate("Form", "автоматика"))
        self.info.setAccessibleName(_translate("Form", "информация"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.info), _translate("Form", "информация"))


class Timer:
    def __init__(self, tick):
        self.tick, self.last = tick, datetime.now()

    def tk(self):
        if (datetime.now() - self.last) > timedelta(seconds=self.tick):
            self.last = datetime.now()
            return True
        return False


class MyWidget(QMainWindow, MainWindow):
    content = {}
    com_buf = []
    ind_l = [8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]
    cmd_buf = []
    ind_c = 0
    serial = None
    mute_fl = False
    wf, gf = False, False
    pp_time = Timer(2)

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.timer = QTimer()
        self.timer.timeout.connect(self.step)
        self.timer.start(2000)
        self.upd_com.clicked.connect(self.upd_coms_f)
        self.hiden_thr = QThread()
        self.com_upd_thr = GetSerialInfo()
        self.com_upd_thr.moveToThread(self.hiden_thr)
        self.com_upd_thr.data.connect(self._upd_coms_f)
        self.con_com.clicked.connect(self.connect_f)
        self.update2.clicked.connect(self.update_modules_f)
        self.clear.clicked.connect(self.output.clear)
        self.send.clicked.connect(self.send_f)
        self.switch1.clicked.connect(self.switch_w)
        self.switch2.clicked.connect(self.switch_g)
        self.upd_coms_f()
        self.led_but.clicked.connect(lambda g: self.cmd_buf.append(f"0#{self.comboBox_2.currentText()}#1#14#0#{self.led_br.value()}"))
        self.buz_but.clicked.connect(lambda g: self.cmd_buf.append(f"0#{self.comboBox_2.currentText()}#1#15#{self.buz_time.value()}#{self.buz_freq.value()}"))
        self.com_sel_ = {0: lambda g: self.comboBox_2.clear(),
                         1: lambda g: self.comboBox_2.addItem(str(g["sender"])),
                         8: lambda g: (self.cmd_buf.append(f"0#{g['sender']}#1#13#0#1") if g['int_data'] > self.spinBox_4.value() and self.ign_fl.isChecked() else (self.cmd_buf.append(f"0#{g['sender']}#1#13#0#0") if g['int_data'] < self.spinBox_9.value() and self.ign_fl.isChecked() else 0), self.mol_1_lcd.display(g["int_data"])),
                         9: lambda g: (self.cmd_buf.append(f"0#{g['sender']}#1#13#0#1") if g['int_data'] > self.spinBox_4.value() and self.ign_fl.isChecked() else (self.cmd_buf.append(f"0#{g['sender']}#1#13#0#0") if g['int_data'] < self.spinBox_9.value() and self.ign_fl.isChecked() else 0), self.mol_2_lcd.display(g["int_data"])),
                         10: lambda g: self.but_1_lcd.display("1111" if g["int_data"] == 1 else "0000"),
                         11: lambda g: self.but_2_lcd.display("1111" if g["int_data"] == 1 else "0000"),
                         16: lambda g: self.aqi_lcd.display(g['int_data']),
                         17: lambda g: self.eco2_lcd.display(g['int_data']),
                         18: lambda g: self.tvoc_lcd.display(g['int_data']),
                         19: lambda g: self.temp_lcd.display(g['data']),
                         20: lambda g: self.hum_lcd.display(g['data']),
                         21: lambda g: self.sv_lcd.display(g['data']),
                         22: lambda g: self.bv_lcd.display(g['data']),
                         23: lambda g: self.is_lcd.display(g['data'])}

    def _upd_coms_f(self, data: dict):
        self.content = data.copy()
        self.com.clear()
        for ke in data.keys():
            self.com.addItem(ke)
        self.hiden_thr.terminate()

    def switch_w(self):
        self.cmd_buf.append(f"0#{self.comboBox_2.currentText()}#1#13#0#{1 if self.wf else 0}")
        self.wf = not self.wf
        self.mute_fl = True

    def switch_g(self):
        self.cmd_buf.append(f"0#{self.comboBox_2.currentText()}#1#12#0#{1 if self.wf else 0}")
        self.gf = not self.gf
        self.mute_fl = True

    def upd_coms_f(self):
        self.hiden_thr.terminate()
        self.hiden_thr.started.connect(self.com_upd_thr.run)
        self.hiden_thr.start()

    def receive(self):
        while self.serial.canReadLine():
            text = self.serial.readLine().data().decode()
            text = text.rstrip('\r\n').split("#")
            self.output.append(f"{datetime.now().strftime('%d.%m.%Y %H:%M:%S')} <-- {' '.join(text)}")
            self.com_buf.append({"target": int(text[1]), "sender": int(text[0]), "is_com": int(text[2]),
                                 "com_cod": int(text[3]), "data": float(text[4]), "int_data": int(text[5])})

    def send_f(self, tt=""):
        if self.upd_pb.value() == 100:
            if tt:
                self.serial.write(tt.encode())
                self.output.append(f"{datetime.now().strftime('%d.%m.%Y %H:%M:%S')} --> {' '.join(tt.split('#'))}")
            else:
                self.serial.write(self.lineEdit.text().encode())
                self.output.append(f"{datetime.now().strftime('%d.%m.%Y %H:%M:%S')} --> {' '.join(self.lineEdit.text().split('#'))}")

    def connect_f(self):
        if self.con_com.text() == "подключиться":
            if self.com.currentText():
                print(self.content[self.com.currentText()])
                self.serial = QtSerialPort.QSerialPort("COM3", baudRate=QtSerialPort.QSerialPort.Baud9600, readyRead=self.receive)
                if not self.serial.isOpen():
                    if not self.serial.open(QIODevice.ReadWrite):
                        self.con_com.setText("подключиться")
                        self.upd_pb.setValue(0)
                        return 0
                    else:
                        self.upd_pb.setValue(100)
                        self.con_com.setText("отключиться")
                        return 1
        if self.serial is not None:
            self.serial.close()
            self.con_com.setText("подключиться")
            self.upd_pb.setValue(0)

    def update_modules_f(self):
        if self.upd_pb.value() == 100:
            self.comboBox_2.clear()
            self.send_f("0#1#1#1#0#0")

    def step(self):
        if self.upd_pb.value() == 100:
            if self.com_buf:
                cp = self.com_buf.pop(0)
                if cp['com_cod'] in self.com_sel_.keys():
                    self.com_sel_[cp['com_cod']](cp)
            if self.comboBox_2.currentText() and not self.mute_fl:
                if self.cmd_buf:
                    self.send_f(self.cmd_buf.pop(0))
                else:
                    self.send_f(f"0#{self.comboBox_2.currentText()}#1#{self.ind_l[self.ind_c]}#0#0")
                    self.ind_c += 1
                    if self.ind_c >= len(self.ind_l):
                        self.ind_c = 0
            else:
                if self.cmd_buf:
                    self.send_f(self.cmd_buf.pop(0))
        if self.pp_time.tk():
            self.mute_fl = False


class GetSerialInfo(QObject):
    """получение информации о видео в параллельном потоке"""
    data = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

    @pyqtSlot()
    def run(self):
        data = {}
        for i in QtSerialPort.QSerialPortInfo.availablePorts():
            if i.isValid():
                ke = f"{i.description()} ({i.portName()})"
                data[ke] = i.portName()
        self.data.emit(data)



def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    ex = MyWidget()
    ex.show()
    sys.excepthook = except_hook
    sys.exit(app.exec_())
    """
    python -m PyQt5.uic.pyuic -x untitled.ui -o main.py
    """