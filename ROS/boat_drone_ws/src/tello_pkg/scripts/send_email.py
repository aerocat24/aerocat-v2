#!/usr/bin/env python3
"""
Send email from a Gmail account with an attachment.
"""

import smtplib
import ssl
import rospy
from std_msgs.msg import String

from email import encoders
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

def send_email(filename):
    try:
        # Create SMTP server
        context = ssl.create_default_context()
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.ehlo()
        server.starttls(context=context)
        server.ehlo()

        # Sender information
        sender_email = 'alertas.tcc.marcusalbano@gmail.com'
        password = 'tvlryemcpovimmld'

        # Recipient information
        receivers = ['alertas.tcc.marcusalbano@gmail.com']

        # Email data
        message = MIMEMultipart()
        message['Subject'] = 'Face Detectada!'
        message['From'] = sender_email
        message['To'] = ','.join(receivers)

        html = 'Uma face foi detectada pelo drone.'
        message.attach(MIMEText(html, 'html'))

        # Set attachment attributes
        attachment = MIMEBase('application', 'octet-stream')

        with open(filename, 'rb') as f:
            attachment.set_payload(f.read())

        encoders.encode_base64(attachment)

        attachment.add_header(
            'Content-Disposition',
            f'attachment; filename={filename}',
        )

        # Attach file to email
        message.attach(attachment)

        # Login to server
        server.login(sender_email, password)

        # Send email
        server.sendmail(sender_email, receivers, message.as_string())
        rospy.loginfo(f"Email sent with attachment: {filename}")

    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        # Close the server
        server.quit()

def file_callback(msg):
    filename = msg.data
    rospy.loginfo(f"Received file: {filename}")
    send_email(filename)

def email_sender_node():
    rospy.init_node('email_sender', anonymous=False)
    rospy.Subscriber('/file_face_detected', String, file_callback)
    rospy.loginfo("Email sender node is running...")
    rospy.spin()

if __name__ == '__main__':
    email_sender_node()
