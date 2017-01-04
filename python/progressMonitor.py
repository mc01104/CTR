# python
import smtplib
import sched
import time
import glob, os
import shutil
import getpass

MESSAGE_FORMAT = "From: %s\r\nTo: %s\r\nCC: %s\r\nSubject: %s\r\n\r\n%s" # %(fromAddr,to,cc, subject,text)

mail = "georgios.fagogenis@gmail.com"
cc_address = "hjhdog1@gmail.com"
subject = "progress report"
passwd = getpass.getpass()
s = sched.scheduler(time.time, time.sleep)
delay = 60 * 60 * 3 #in seconds
addresses = [mail, cc_address]


def compose_progress_report():
    os.chdir("C:/Users/CTR/Desktop/Shapes")
    msg =""
    for file in glob.glob("*.stl"):
        size = os.path.getsize(file) >> 20
        msg += "Filename:%s, Size: %s Mb\n" % (file, size)
        new_filename = "processed/" + file
        shutil.move(file, new_filename)
    return msg


def send_progress_report():
    msg = compose_progress_report()
    message = MESSAGE_FORMAT % ('', mail, cc_address, subject, msg)

    server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
    server.login(mail, passwd)
    server.sendmail(mail, addresses, message)
    server.quit()

def do_something(sc):
    send_progress_report()
    s.enter(delay, 1, do_something, (sc,))

s.enter(delay, 1, do_something, (s,))
s.run()


