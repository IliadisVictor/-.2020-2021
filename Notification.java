package com.company;

public class Notification {
    private int Nottype;
    private int RecipientId;
    private int NotificationId;


    public int getType() {
        return this.Nottype;
    }

    public void setType(int type) {
        this.Nottype = type;
    }

    public int getRecipientId() {
        return this.RecipientId;
    }

    public void setRecipientId(int recipientId) {
        this.RecipientId = recipientId;
    }

    public int getNotificationId() {
        return this.NotificationId;
    }

    public void setNotificationId(int notificationId) {
        this.NotificationId = notificationId;
    }


    public Notification(int type, int recipientId) {
        this.Nottype = type;
        this.RecipientId = recipientId;
        this.NotificationId = 0;
    }
}
