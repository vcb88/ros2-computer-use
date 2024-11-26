import { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import { RandomMessage } from '../types/messages';

export const useROS = () => {
    const [connected, setConnected] = useState(false);
    const [messageCount, setMessageCount] = useState(0);
    const [lastMessage, setLastMessage] = useState<RandomMessage | null>(null);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        // Initialize ROS connection
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        // Setup connection handlers
        ros.on('connection', () => {
            console.log('Connected to websocket server.');
            setConnected(true);
            setError(null);
        });

        ros.on('error', (error) => {
            console.log('Error connecting to websocket server:', error);
            setError('Error connecting to ROS');
            setConnected(false);
        });

        ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            setConnected(false);
        });

        // Subscribe to the random_data topic
        const topic = new ROSLIB.Topic({
            ros: ros,
            name: '/random_data',
            messageType: 'std_msgs/String'
        });

        topic.subscribe((message: ROSLIB.Message) => {
            try {
                const data = JSON.parse((message as any).data);
                setLastMessage(data);
                setMessageCount(prev => prev + 1);
            } catch (e) {
                console.error('Error parsing message:', e);
                setError('Error parsing message');
            }
        });

        // Cleanup on unmount
        return () => {
            topic.unsubscribe();
            ros.close();
        };
    }, []);

    return {
        connected,
        messageCount,
        lastMessage,
        error
    };
};