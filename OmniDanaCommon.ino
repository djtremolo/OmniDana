void DumpMessage(uint8_t* prefix, uint8_t* postfix, uint8_t* buf, size_t len)
{
  size_t i;
  
  Serial.print(prefix);
  Serial.print(xReceivedBytes, DEC);
  Serial.print(postfix);

  for(i=0; i<len; i++)
  {
    Serial.print(buf[i], HEX);
  }

  Serial.println(".");
}
