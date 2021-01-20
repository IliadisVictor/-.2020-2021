package com.company;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

public class NotificationDao {

    public void DeleteNotification(int NotId) throws Exception {
        Database dataObj = new Database();
        Connection con = dataObj.getConnection();
        String number = String.valueOf(NotId);
        try {
            PreparedStatement st = con.prepareStatement("DELETE FROM Notifications WHERE Notificationid = ?");
            st.setString(1, number);
            st.executeUpdate();
        } catch (Exception e) {
            throw new Exception("Something went wrong " + e.getMessage());
        } finally {
            con.close();
        }
    }

    public ArrayList<Notification> getNotifications(int UserId) throws Exception {
        Database dataObj = new Database();
        Connection con = dataObj.getConnection();
        String number = String.valueOf(UserId);
        try {
            ArrayList<Notification> notifications = new ArrayList<Notification>();
            String query = "SELECT * FROM Notifications where RecepientId=?";
			PreparedStatement statement = con.prepareStatement(query);
            statement.setString(1, number);
			ResultSet rs = statement.executeQuery();
			while (rs.next()) {
			   int notificationid=rs.getInt(1);
			   int nottype=rs.getInt(2);
			   int Recepientid=rs.getInt(3);
			   Notification newnot=new Notification(nottype,Recepientid);
			   newnot.setNotificationId(notificationid);
			   notifications.add(newnot);
            }
            return notifications;
        } catch (Exception e) {
			throw new Exception("Something went wrong " + e.getMessage());
		} finally {
			con.close();
		}
    }

    public void AddNotification(int Type,int RecepientId  ) throws Exception {
        Database dataObj = new Database();
        Connection con = dataObj.getConnection();

        try {
            String query = " insert into Notifications (NotType,RecepientId)"
                    + " values (?, ?)";

            PreparedStatement st = con.prepareStatement(query);
            st.setInt(1, Type);
            st.setInt(2, RecepientId);

            st.executeUpdate();
        } catch (Exception e) {
            throw new Exception("Something went wrong " + e.getMessage());
        } finally {
            con.close();
        }

    }
}

