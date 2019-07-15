/*
 * Copyright (c) 2019 Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import org.eclipse.lyo.trs.client.handlers.IProviderEventHandler;
import org.eclipse.lyo.trs.client.model.BaseMember;
import org.eclipse.lyo.trs.client.model.ChangeEventMessageTR;
import org.eclipse.lyo.trs.client.mqtt.MqttProviderHandler;
import org.eclipse.lyo.trs.client.mqtt.MqttTrsEventListener;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig;

public class TRSManager {
    private final static Logger log = LoggerFactory.getLogger(TRSManager.class);

    public static void initTRSClient(final MqttClient mqttClient) throws MqttException {
        final IProviderEventHandler eventHandler = new LoggingProviderHandler();
        final MqttProviderHandler providerHandler = new MqttProviderHandler(eventHandler);
        final IMqttMessageListener messageListener = new MqttTrsEventListener(providerHandler);
        final String topic = AdaptorHelper.p(WhcConfig.MQTT_TOPIC_PROP);

        mqttClient.subscribe(topic, messageListener);

        log.info("TRS event listener registered on the '{}' topic", topic);
    }

    private static class LoggingProviderHandler implements IProviderEventHandler {
        private final static Logger log = LoggerFactory.getLogger(LoggingProviderHandler.class);

        @Override
        public void finishCycle() {

        }

        @Override
        public void handleBaseMember(final BaseMember baseMember) {

        }

        @Override
        public void handleChangeEvent(final ChangeEventMessageTR eventMessageTR) {
            log.info("New CE received: {}", eventMessageTR.getChangeEvent());
        }

        @Override
        public void rebase() {

        }
    }
}
