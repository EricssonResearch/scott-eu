<!DOCTYPE html>
<%--Start of user code "Copyright"
--%>
<%--
 Copyright (c) 2011, 2012, 2017 IBM Corporation and others.

 All rights reserved. This program and the accompanying materials
 are made available under the terms of the Eclipse Public License v1.0
 and Eclipse Distribution License v. 1.0 which accompanies this distribution.

 The Eclipse Public License is available at http://www.eclipse.org/legal/epl-v10.html
 and the Eclipse Distribution License is available at
 http://www.eclipse.org/org/documents/edl-v10.php.

 Contributors:

  Sam Padgett     - initial API and implementation
  Michael Fiedler - adapted for OSLC4J
  Jad El-khoury   - initial implementation of code generator (422448)
  Frédéric Loiret - Switch the template to Bootstrap (519699)

 This file is generated by org.eclipse.lyo.oslc4j.codegenerator
--%>
<%--End of user code--%>

<%--Start of user code "body"
--%>
<%--TODO: Replace/adjust this default content as necessary.
All manual changes in this "protected" user code area will NOT be overwritten upon subsequent code generations.
To revert to the default generated content, delete all content in this file, and then re-generate.
--%>

<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c" %>

<%@ page import="java.net.URI" %>
<%@ page import="java.net.URLEncoder" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.Service" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.ServiceProvider" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.Dialog" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.CreationFactory" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.ResourceShape" %>
<%@ page import="org.eclipse.lyo.oslc4j.core.model.QueryCapability" %>

<%@ page contentType="text/html" language="java" pageEncoding="UTF-8" %>

<%
ServiceProvider serviceProvider = (ServiceProvider)request.getAttribute("serviceProvider");
Service[] services = (Service[])request.getAttribute("services");
%>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title><%= serviceProvider.getTitle() %></title>
    <link href="<c:url value="/static/css/bootstrap-4.0.0-beta.min.css"/>" rel="stylesheet">
    <link href="<c:url value="/static/css/adaptor.css"/>" rel="stylesheet">

    <script src="<c:url value="/static/js/jquery-3.2.1.min.js"/>"></script>
    <script src="<c:url value="/static/js/popper-1.11.0.min.js"/>"></script>
    <script src="<c:url value="/static/js/bootstrap-4.0.0-beta.min.js"/>"></script>
</head>
<body>

  <nav class="navbar sticky-top navbar-light bg-light">
    <div class="container">
      <a class="navbar-brand" href="<c:url value="/services/catalog/singleton"/>">Twin</a>
    </div>
  </nav>

  <div class="container">
    <div class="page-header">
      <h1><%= serviceProvider.getTitle() %></h1>
      <p><%= serviceProvider.getDescription() %></p>


    </div>
    <%for (int serviceIndex = 0; serviceIndex < services.length; serviceIndex++) {%>
    <h2>Service &#35;<%=serviceIndex%></h2>
    <% Dialog[] selectionDialogs = services[serviceIndex].getSelectionDialogs();%>
    <% if(selectionDialogs.length > 0) {%>
    <h4>Resource Selector Dialogs</h4>
    <%}%>
    <%
        for (int selectionDialogIndex = 0; selectionDialogIndex < selectionDialogs.length; selectionDialogIndex++) {
            String selectionDialog = selectionDialogs[selectionDialogIndex].getDialog().toString();
    %>
    <p>
        <a href="<%= selectionDialog %>"><%= selectionDialog %></a>
    (<a href="<%= request.getContextPath() %>/se/ericsson/cf/scott/sandbox/twin/selectiondialogsampleclient.jsp?selectionUri=<%= URLEncoder.encode(selectionDialog.toString(), "UTF-8") %>">sample client</a>)
    </p>
    <%}%>
    <% Dialog[] creationDialogs = services[serviceIndex].getCreationDialogs(); %>
    <% if(creationDialogs.length > 0) {%>
    <h4>Resource Creator Dialogs</h4>
    <%}%>
    <%
        for (int creationDialogIndex = 0; creationDialogIndex < creationDialogs.length; creationDialogIndex++) {
            String creationDialog = creationDialogs[creationDialogIndex].getDialog().toString();
    %>
    <p>
        <a href="<%= creationDialog %>"><%= creationDialog %></a>
    (<a href="<%= request.getContextPath() %>/se/ericsson/cf/scott/sandbox/twin/creationdialogsampleclient.jsp?creationUri=<%= URLEncoder.encode(creationDialog.toString(), "UTF-8") %>">sample client</a>)
    </p>
    <%}%>
    <% CreationFactory[] creationFactories = services[serviceIndex].getCreationFactories(); %>
    <% if(creationFactories.length > 0) {%>
    <h4>Resource Creation Factories</h4>
    <%}%>
    <%
        for (int creationFactoryIndex = 0; creationFactoryIndex < creationFactories.length; creationFactoryIndex++) {
            String creationFactory = creationFactories[creationFactoryIndex].getCreation().toString();
    %>
    <p><a href="<%= creationFactory %>"><%= creationFactory %></a></p>
    <%}%>
    <% QueryCapability[] queryCapabilities= services[serviceIndex].getQueryCapabilities(); %>
    <% if(queryCapabilities.length > 0) {%>
    <h4>Resource Query Capabilities</h4>
    <%}%>
    <%
        for (int queryCapabilityIndex = 0; queryCapabilityIndex < queryCapabilities.length; queryCapabilityIndex++) {
            QueryCapability qc = queryCapabilities[queryCapabilityIndex];
            String queryCapability = qc.getQueryBase().toString();
    %>
    <p><a href="<%= queryCapability %>"><%= qc.getTitle() %> (<code><%= qc.getQueryBase() %></code>)</a></p>
    <%}%>
    <% if(creationFactories.length > 0) {%>
    <h4>Creation Resource Shapes</h4>
    <%
        for (int creationFactoryIndex = 0; creationFactoryIndex < creationFactories.length; creationFactoryIndex++) {
            URI[] creationShapes = creationFactories[creationFactoryIndex].getResourceShapes();
            String creationShape = creationShapes[0].toString();
    %>
    <p><a href="<%= creationShape %>"><%= creationShape %></a></p>
    <%}%>
    <%}%>
    <% if(queryCapabilities.length > 0) {%>
    <h4>Query Resource Shapes</h4>
    <%
        for (int queryCapabilityIndex = 0; queryCapabilityIndex < queryCapabilities.length; queryCapabilityIndex++) {
            String queryShape = queryCapabilities[queryCapabilityIndex].getResourceShape().toString();
    %>
    <p><a href="<%= queryShape %>"><%= queryShape %></a></p>
    <%}%>
    <%}%>

    <c:if test="<%=serviceIndex + 1 < services.length%>">
      <hr>
    </c:if>
    <%}%>
  </div>
  <footer class="footer">
    <div class="container">
      <p class="text-muted">
        OSLC Adaptor was generated using <a href="http://eclipse.org/lyo">Eclipse Lyo</a>.
      </p>
    </div>
  </footer>
</body>
</html>
<%--End of user code--%>
